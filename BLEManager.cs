// ═══════════════════════════════════════════════════════════════════════════
//  BLEManager.cs
//  Sensor-Fusion Tracking System — AIR Group, ESI-UCLM
// ═══════════════════════════════════════════════════════════════════════════
//
//  PURPOSE
//  -------
//  Singleton manager that handles the complete Bluetooth Low Energy (BLE)
//  lifecycle for custom rehabilitation smart objects ("cubos sensoriales"):
//
//    1. Android permission requests (BLUETOOTH_SCAN, BLUETOOTH_CONNECT, …)
//    2. BLE hardware initialisation in central-only mode
//    3. Automatic scanning for peripherals whose advertised names match a
//       configurable target list (e.g. "RehabRed", "RehabBlue", "RehabGreen")
//    4. GATT connection establishment with auto-reconnection on disconnect
//    5. Subscription to three GATT characteristics per peripheral:
//         • IMU quaternion  (16 bytes = 4 × IEEE 754 float, w-x-y-z order)
//         • FSR grip force  (1, 2, or 4 bytes depending on firmware)
//         • Battery level   (standard BT SIG 0x180F / 0x2A19)
//    6. Coordinate-system conversion from the sensor's right-handed frame
//       to Unity's left-handed frame
//
//  DATA FLOW
//  ---------
//  When a GATT notification arrives, the corresponding DeviceData instance
//  is updated in-place and the static event OnDataUpdated is raised.
//  FusionSystemManager listens to OnConnected to spawn FusionTracker
//  instances; FusionTracker reads DeviceData every frame in LateUpdate.
//
//  SCANNING STRATEGY
//  -----------------
//  Scanning pauses during a connection attempt (some Android BLE stacks
//  cannot scan and connect simultaneously) and resumes once the connection
//  is established, to discover any remaining target devices.
//
//  DEPENDENCIES
//  ------------
//  • BluetoothLEHardwareInterface — third-party Unity BLE plugin
//  • Android 12+ runtime permissions model
//
// ═══════════════════════════════════════════════════════════════════════════

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Android;

// ─────────────────────────────────────────────────────────────────────────
//  DeviceData — Immutable snapshot for a single BLE peripheral
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// Runtime data snapshot for one connected BLE rehabilitation device.
///
/// <para>Each connected peripheral has exactly one <see cref="DeviceData"/>
/// instance stored in <see cref="BLEManager.connectedDevices"/>. Fields are
/// updated in-place every time a GATT notification arrives.</para>
///
/// <list type="bullet">
///   <item><see cref="id"/>          — BLE MAC address (unique identifier).</item>
///   <item><see cref="name"/>        — Advertised device name (e.g. "RehabRed").</item>
///   <item><see cref="battery"/>     — Battery percentage (0–100 %).</item>
///   <item><see cref="grip"/>        — FSR grip force (raw integer value).</item>
///   <item><see cref="orientation"/> — IMU quaternion in Unity's left-handed
///         coordinate system.  <c>w = 0</c> indicates that no IMU data has
///         been received yet.</item>
/// </list>
///
/// <para>The <c>[NonSerialized]</c> freshness flags (<see cref="HasImuData"/>,
/// <see cref="LastImuUpdateTime"/>, etc.) allow consumers to check whether
/// valid data has arrived at least once and how recently.</para>
/// </summary>
[Serializable]
public sealed class DeviceData
{
    /// <summary>BLE MAC address — primary key in <see cref="BLEManager.connectedDevices"/>.</summary>
    public string id;

    /// <summary>Human-readable advertised name (e.g. "RehabRed").</summary>
    public string name;

    /// <summary>Battery percentage (0–100). Updated from the Battery Service.</summary>
    public int battery;

    /// <summary>
    /// Raw grip force value from the FSR sensor. The integer range depends
    /// on the firmware payload format (see <see cref="BLEManager.ParseGripValue"/>).
    /// </summary>
    public int grip;

    /// <summary>
    /// IMU orientation in Unity's left-handed coordinate system.
    /// A <c>w</c> value of <c>0</c> means no IMU data has been received yet.
    /// </summary>
    public Quaternion orientation;

    // ── Runtime-only freshness flags (not serialised) ────────────────

    /// <summary><c>true</c> once at least one IMU notification has been parsed.</summary>
    [NonSerialized] public bool HasImuData;

    /// <summary><see cref="Time.realtimeSinceStartup"/> of the most recent IMU update.</summary>
    [NonSerialized] public float LastImuUpdateTime;

    /// <summary><c>true</c> once at least one FSR notification has been parsed.</summary>
    [NonSerialized] public bool HasGripData;

    /// <summary><see cref="Time.realtimeSinceStartup"/> of the most recent FSR update.</summary>
    [NonSerialized] public float LastGripUpdateTime;
}

// ─────────────────────────────────────────────────────────────────────────
//  BLEManager — Singleton BLE lifecycle manager
// ─────────────────────────────────────────────────────────────────────────

/// <summary>
/// Singleton that manages BLE scanning, connection, and GATT subscriptions
/// for rehabilitation smart objects.
///
/// <para><b>Usage:</b> Attach to a single GameObject in the initial scene.
/// The object is marked <see cref="DontDestroyOnLoad"/> so connections
/// survive scene transitions. Access the instance via
/// <see cref="BLEManager.Instance"/>.</para>
///
/// <para><b>Events:</b></para>
/// <list type="bullet">
///   <item><see cref="OnConnected"/>    — peripheral finished connecting (MAC).</item>
///   <item><see cref="OnDisconnected"/> — peripheral disconnected unexpectedly (MAC).</item>
///   <item><see cref="OnDataUpdated"/>  — characteristic notification received (MAC).</item>
/// </list>
/// </summary>
public sealed class BLEManager : MonoBehaviour
{
    // ══════════════════════════════════════════════════════════════════
    //  Singleton
    // ══════════════════════════════════════════════════════════════════

    /// <summary>Global access point to the single BLEManager instance.</summary>
    public static BLEManager Instance { get; private set; }

    // ══════════════════════════════════════════════════════════════════
    //  Inspector Configuration
    // ══════════════════════════════════════════════════════════════════

    [Header("Target Devices (exact BLE advertised names)")]
    [Tooltip("Peripheral names the scanner will connect to automatically.")]
    public List<string> targetDeviceNames = new()
    {
        "RehabRed",
        "RehabBlue",
        "RehabGreen"
    };

    [Header("MAC → Color (fijo)")]
    [Tooltip("Si true, usa macColorMap para resolver nombres ignorando el nombre BLE.")]
    public bool enableMacLookup = true;

    /// <summary>Mapeo fijo MAC→nombre. Tiene prioridad absoluta sobre el nombre BLE.</summary>
    private static readonly Dictionary<string, string> macColorMap = new()
    {
        { "E4:32:5A:81:6E:64", "RehabGreen" },
        { "FE:13:2A:ED:BC:78", "RehabBlue"  },
        { "D4:91:B7:47:30:BD", "RehabRed"   },
    };

    [Header("GATT UUIDs")]
    [Tooltip("Custom service UUID exposed by the rehab cube firmware.")]
    public string targetServiceUUID = "12345678-1234-5678-1234-56789abcdef0";

    [Tooltip("Characteristic UUID for the IMU quaternion (16 bytes, w-x-y-z).")]
    public string imuCharacteristicUUID = "12345678-1234-5678-1234-56789abcdef1";

    [Tooltip("Characteristic UUID for the FSR grip force (1–4 bytes).")]
    public string fsrCharacteristicUUID = "12345678-1234-5678-1234-56789abcdef2";

    // ══════════════════════════════════════════════════════════════════
    //  Runtime State
    // ══════════════════════════════════════════════════════════════════

    [Header("Connected Devices (read-only at runtime)")]
    public Dictionary<string, DeviceData> connectedDevices = new();

    // ══════════════════════════════════════════════════════════════════
    //  Events
    // ══════════════════════════════════════════════════════════════════

    /// <summary>Raised when a target peripheral finishes connecting. Passes the MAC address.</summary>
    public static event Action<string> OnConnected;

    /// <summary>Raised when a peripheral disconnects unexpectedly. Passes the MAC address.</summary>
    public static event Action<string> OnDisconnected;

    /// <summary>Raised every time a characteristic notification arrives. Passes the MAC address.</summary>
    public static event Action<string> OnDataUpdated;

    // ══════════════════════════════════════════════════════════════════
    //  Constants
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Android runtime permissions required for BLE on Android 12+.
    /// ACCESS_FINE_LOCATION is still needed on some device/SDK combinations.
    /// </summary>
    private static readonly string[] RequiredAndroidPermissions =
    {
        "android.permission.BLUETOOTH_SCAN",
        "android.permission.BLUETOOTH_CONNECT",
        "android.permission.ACCESS_FINE_LOCATION"
    };

    /// <summary>Standard Bluetooth SIG Battery Service UUID (0x180F).</summary>
    private const string BatteryServiceUuid = "180f";

    /// <summary>Standard Bluetooth SIG Battery Level Characteristic UUID (0x2A19).</summary>
    private const string BatteryCharacteristicUuid = "2a19";

    // ══════════════════════════════════════════════════════════════════
    //  Private Fields
    // ══════════════════════════════════════════════════════════════════

    /// <summary>Guards against double-initialisation on scene restarts.</summary>
    private bool _initialized;

    /// <summary>Prevents concurrent scan requests.</summary>
    private bool _isScanning;

    /// <summary>
    /// MAC addresses currently in the process of connecting (not yet in
    /// <see cref="connectedDevices"/>). Prevents duplicate connection attempts
    /// when multiple scan callbacks fire before the first connect completes.
    /// </summary>
    private readonly HashSet<string> _pendingConnections = new();

    // ══════════════════════════════════════════════════════════════════
    //  Lifecycle
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Enforces the singleton pattern: if a duplicate instance exists, the
    /// newcomer destroys itself; otherwise the first instance persists across
    /// scene loads via <see cref="DontDestroyOnLoad"/>.
    /// </summary>
    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    /// <summary>
    /// Requests Android BLE permissions, then schedules hardware
    /// initialisation after a 1-second delay to let the permission dialog
    /// complete before any Bluetooth library calls are made.
    /// </summary>
    private void Start()
    {
        StartCoroutine(InitWithPermissions());
    }

    /// <summary>
    /// Requests Android BLE permissions and waits until all are granted
    /// (or times out after 30 s) before initialising the BLE stack.
    /// This prevents the first-run race condition where BLE init ran
    /// before the user had tapped "Allow" on the permission dialogs.
    /// </summary>
    private IEnumerator InitWithPermissions()
    {
        RequestAndroidPermissions();

        // Poll hasta que todos los permisos estén concedidos.
        float timeout = 30f;
        float waited = 0f;
        while (!AllPermissionsGranted() && waited < timeout)
        {
            yield return new WaitForSeconds(0.5f);
            waited += 0.5f;
        }

        if (!AllPermissionsGranted())
        {
            Debug.LogError("[BLE] Permisos BLE no concedidos tras 30 s. " +
                           "El Bluetooth no se inicializará.");
            yield break;
        }

        // Pequeña pausa post-concesión para que Android estabilice el stack.
        yield return new WaitForSeconds(0.5f);
        InitializeBluetooth();
    }

    // ══════════════════════════════════════════════════════════════════
    //  Permission Handling
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Iterates through <see cref="RequiredAndroidPermissions"/> and requests
    /// any that have not yet been authorised. Each permission is checked
    /// individually because the Android API does not support reliable batch
    /// requests across all Unity versions.
    /// </summary>
    private static void RequestAndroidPermissions()
    {
        foreach (string permission in RequiredAndroidPermissions)
        {
            if (!Permission.HasUserAuthorizedPermission(permission))
                Permission.RequestUserPermission(permission);
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Initialisation
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Returns <c>true</c> when every permission in
    /// <see cref="RequiredAndroidPermissions"/> has been authorised.
    /// </summary>
    private static bool AllPermissionsGranted()
    {
        foreach (string p in RequiredAndroidPermissions)
            if (!Permission.HasUserAuthorizedPermission(p))
                return false;
        return true;
    }

    /// <summary>
    /// Initialises the BLE hardware interface in central-only mode (no
    /// advertising). On success, the first automatic scan is scheduled
    /// after a short settling delay.
    /// </summary>
    private void InitializeBluetooth()
    {
        if (_initialized) return;

        BluetoothLEHardwareInterface.Initialize(
            asCentral: true,
            asPeripheral: false,
            action: () =>
            {
                _initialized = true;
                Invoke(nameof(StartAutoScan), 1.0f);
            },
            errorAction: error => Debug.LogError($"[BLE] Initialize error: {error}"));
    }

    // ══════════════════════════════════════════════════════════════════
    //  Scanning
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Begins scanning for peripherals whose advertised names match
    /// <see cref="targetDeviceNames"/>. When a target is discovered, scanning
    /// stops immediately and the connection process begins.
    ///
    /// <para>Service filter is <c>null</c> (scan all peripherals) because
    /// different cube hardware revisions may not advertise the same UUID.</para>
    /// </summary>
    private void StartAutoScan()
    {
        if (_isScanning) return;
        if (connectedDevices.Count >= targetDeviceNames.Count) return;

        _isScanning = true;

        BluetoothLEHardwareInterface.ScanForPeripheralsWithServices(
            serviceUUIDs: null,
            action: (address, name) =>
            {
                if (string.IsNullOrWhiteSpace(name)) return;
                if (!targetDeviceNames.Contains(name)) return;
                if (connectedDevices.ContainsKey(address) || _pendingConnections.Contains(address)) return;

                Debug.Log($"[BLE] Target found: {name} ({address})");

                _pendingConnections.Add(address);

                // Stop scanning before connecting — some Android stacks cannot do both.
                BluetoothLEHardwareInterface.StopScan();
                _isScanning = false;

                ConnectToDevice(address, name);
            },
            actionAdvertisingInfo: null);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Connection
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Initiates a GATT connection to the peripheral at <paramref name="address"/>.
    /// </summary>
    /// <param name="address">BLE MAC address of the peripheral.</param>
    /// <param name="deviceName">Advertised name, used for logging and name resolution.</param>
    private void ConnectToDevice(string address, string deviceName)
    {
        BluetoothLEHardwareInterface.ConnectToPeripheral(
            address,
            connectAction: addr => HandlePeripheralConnected(addr, deviceName),
            serviceAction: null,
            characteristicAction: null,
            disconnectAction: HandlePeripheralDisconnected);
    }

    /// <summary>
    /// Called when a peripheral connection succeeds. Creates a fresh
    /// <see cref="DeviceData"/>, fires <see cref="OnConnected"/>, subscribes
    /// to GATT characteristics, and resumes scanning for remaining targets.
    ///
    /// <para><b>Android name-cache workaround:</b> Android OS sometimes caches
    /// the advertised name of the first connected device and incorrectly
    /// reports it for subsequent devices with similar hardware. If a name
    /// collision is detected, the code assigns the first unoccupied target
    /// name instead.</para>
    /// </summary>
    private void HandlePeripheralConnected(string address, string deviceName)
    {
        _pendingConnections.Remove(address);
        connectedDevices.Remove(address);  // Remove any stale entry first.

        // ── MAC auto-learn: resolución fiable de nombre ─────────────
        string finalName = ResolveNameByMac(address, deviceName);

        var device = new DeviceData
        {
            id = address,
            name = finalName,
            battery = 100,
            grip = 0,
            orientation = Quaternion.identity,
            HasImuData = false,
            HasGripData = false
        };

        connectedDevices.Add(address, device);

        Debug.Log($"[BLE] Connected: {finalName} ({address}) — Raw OS name: {deviceName}");
        OnConnected?.Invoke(address);

        StartCoroutine(SubscribeToCharacteristics(address));
        Invoke(nameof(StartAutoScan), 1.5f);
    }

    /// <summary>
    /// Handles an unexpected peripheral disconnection: removes the entry,
    /// fires <see cref="OnDisconnected"/>, and restarts scanning so the
    /// device can auto-reconnect.
    /// </summary>
    private void HandlePeripheralDisconnected(string address)
    {
        _pendingConnections.Remove(address);

        string deviceName = connectedDevices.TryGetValue(address, out DeviceData data)
            ? data.name
            : address;

        connectedDevices.Remove(address);

        Debug.Log($"[BLE] Disconnected: {deviceName}");
        OnDisconnected?.Invoke(address);

        if (!_isScanning)
            Invoke(nameof(StartAutoScan), 1.0f);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Characteristic Subscription
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Subscribes to all three GATT characteristics in sequence: FSR → IMU →
    /// Battery. Short delays between subscriptions allow the peripheral's BLE
    /// stack to settle after each notification enable.
    /// </summary>
    private IEnumerator SubscribeToCharacteristics(string address)
    {
        yield return new WaitForSeconds(1.5f);  // Wait for GATT service discovery.

        string serviceUuid = targetServiceUUID.ToLowerInvariant();
        string fsrUuid = fsrCharacteristicUUID.ToLowerInvariant();
        string imuUuid = imuCharacteristicUUID.ToLowerInvariant();

        SubscribeTo(address, serviceUuid, fsrUuid);
        yield return new WaitForSeconds(0.2f);

        SubscribeTo(address, serviceUuid, imuUuid);
        yield return new WaitForSeconds(0.2f);

        SubscribeTo(address, BatteryServiceUuid, BatteryCharacteristicUuid);
    }

    /// <summary>
    /// Subscribes to a single GATT characteristic and routes all incoming
    /// notifications to <see cref="HandleCharacteristicNotification"/>.
    /// </summary>
    private void SubscribeTo(string address, string serviceUUID, string characteristicUUID)
    {
        BluetoothLEHardwareInterface.SubscribeCharacteristicWithDeviceAddress(
            address,
            serviceUUID,
            characteristicUUID,
            notificationAction: (_, _) => { },  // Subscription confirmed — no action needed.
            action: (addr, characteristic, bytes) =>
                HandleCharacteristicNotification(addr, characteristic, bytes));
    }

    // ══════════════════════════════════════════════════════════════════
    //  Data Reception
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Central notification handler for all GATT characteristics. Routes the
    /// incoming byte payload to the appropriate parser based on UUID:
    ///
    /// <list type="bullet">
    ///   <item>FSR UUID → <see cref="ParseGripValue"/> → <see cref="DeviceData.grip"/></item>
    ///   <item>IMU UUID (≥ 16 bytes) → <see cref="ParseImuQuaternion"/> → <see cref="DeviceData.orientation"/></item>
    ///   <item>Battery UUID → raw byte → <see cref="DeviceData.battery"/></item>
    /// </list>
    /// </summary>
    private void HandleCharacteristicNotification(string address, string characteristicUUID, byte[] data)
    {
        if (!connectedDevices.TryGetValue(address, out DeviceData device)) return;
        if (data == null || data.Length == 0) return;

        string uuid = characteristicUUID.ToLowerInvariant();

        if (uuid.Contains(fsrCharacteristicUUID.ToLowerInvariant()))
        {
            device.grip = ParseGripValue(data);
            device.HasGripData = true;
            device.LastGripUpdateTime = Time.realtimeSinceStartup;
        }
        else if (uuid.Contains(imuCharacteristicUUID.ToLowerInvariant()) && data.Length >= 16)
        {
            device.orientation = ParseImuQuaternion(data).normalized;
            device.HasImuData = true;
            device.LastImuUpdateTime = Time.realtimeSinceStartup;
        }
        else if (uuid.Contains(BatteryCharacteristicUuid))
        {
            device.battery = data[0];
        }

        OnDataUpdated?.Invoke(address);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Parsing Helpers
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Parses grip force from a variable-length byte payload.
    /// Supports three firmware formats:
    ///   • 4 bytes → 32-bit signed integer (full FSR range).
    ///   • 2 bytes → 16-bit unsigned integer.
    ///   • 1 byte  → raw byte value (0–255).
    /// </summary>
    private static int ParseGripValue(byte[] data)
    {
        if (data.Length >= 4) return BitConverter.ToInt32(data, 0);
        if (data.Length == 2) return BitConverter.ToUInt16(data, 0);
        return data[0];
    }

    /// <summary>
    /// Decodes a 16-byte IMU quaternion (four IEEE 754 floats in w-x-y-z
    /// order, little-endian) and converts from the sensor's right-handed
    /// coordinate system to Unity's left-handed system.
    ///
    /// <para><b>Coordinate mapping:</b>
    /// Unity(x, y, z, w) = (−sensorY, −sensorZ, sensorX, sensorW).</para>
    ///
    /// <para><b>Byte layout:</b> w at offset 0, x at 4, y at 8, z at 12.</para>
    /// </summary>
    private static Quaternion ParseImuQuaternion(byte[] data)
    {
        float w = BitConverter.ToSingle(data, 0);
        float x = BitConverter.ToSingle(data, 4);
        float y = BitConverter.ToSingle(data, 8);
        float z = BitConverter.ToSingle(data, 12);

        return new Quaternion(-y, -z, x, w);
    }

    // ══════════════════════════════════════════════════════════════════
    //  Public Accessors
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Returns the <see cref="DeviceData"/> for the given MAC address,
    /// or <c>null</c> if the device is not currently connected.
    /// </summary>
    public DeviceData GetDeviceData(string id)
    {
        if (string.IsNullOrWhiteSpace(id)) return null;
        connectedDevices.TryGetValue(id, out DeviceData device);
        return device;
    }

    /// <summary>
    /// Looks up a connected device by its advertised name (case-insensitive).
    /// Returns <c>null</c> if no match is found.
    /// Linear search is acceptable because the connected device count is small (≤ 5).
    /// </summary>
    public DeviceData GetDeviceByName(string deviceName)
    {
        if (string.IsNullOrWhiteSpace(deviceName)) return null;

        foreach (var kvp in connectedDevices)
        {
            if (string.Equals(kvp.Value.name, deviceName, StringComparison.OrdinalIgnoreCase))
                return kvp.Value;
        }

        return null;
    }

    // ══════════════════════════════════════════════════════════════════
    //  MAC Auto-Learn
    // ══════════════════════════════════════════════════════════════════

    /// <summary>
    /// Resuelve el nombre lógico de un dispositivo a partir de su MAC.
    /// <para>Prioridad: 1) MAC guardada en PlayerPrefs, 2) nombre BLE si
    /// no hay conflicto, 3) primer hueco libre (fallback legacy).</para>
    /// Si el nombre se resuelve bien, guarda MAC→nombre para futuras sesiones.
    /// </summary>
    private string ResolveNameByMac(string address, string deviceName)
    {
        // 1) Buscar en tabla fija de MACs
        if (enableMacLookup && macColorMap.TryGetValue(address.ToUpper(), out string mapped))
        {
            Debug.Log($"[BLE] MAC {address} → {mapped} (OS decía: {deviceName})");
            return mapped;
        }

        // 2) MAC desconocida → confiar en el nombre BLE si no hay conflicto
        if (targetDeviceNames.Contains(deviceName) && GetDeviceByName(deviceName) == null)
            return deviceName;

        // 3) Fallback: primer hueco libre
        foreach (string target in targetDeviceNames)
        {
            if (GetDeviceByName(target) == null)
            {
                Debug.LogWarning($"[BLE] MAC {address} desconocida, asignado a hueco: {target}");
                return target;
            }
        }

        return deviceName;
    }

    // ══════════════════════════════════════════════════════════════════
    //  Limpieza al salir
    // ══════════════════════════════════════════════════════════════════

    private void OnApplicationQuit()
    {
        Debug.Log("[BLE] Cerrando aplicación: Desconectando dispositivos...");

        // Detener el escaneo si estaba activo
        if (_isScanning)
        {
            BluetoothLEHardwareInterface.StopScan();
        }

        // Desconectar todos los dispositivos activos
        foreach (string address in connectedDevices.Keys)
        {
            BluetoothLEHardwareInterface.DisconnectPeripheral(address, null);
        }

        // Desinicializar el hardware Bluetooth
        BluetoothLEHardwareInterface.DeInitialize(() => {
            Debug.Log("[BLE] Hardware Bluetooth desinicializado correctamente.");
        });
    }

    private void OnDestroy()
    {
        // Por si el objeto se destruye sin cerrar la app
        OnApplicationQuit();
    }
}