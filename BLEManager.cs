using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Android;

[Serializable]
public sealed class DeviceData
{
    // Paquete en runtime del objeto inteligente. BLE aporta identidad, FSR, bateria,
    // orientacion IMU y movimiento; el resto del sistema no necesita conocer el firmware.
    // Las marcas Has* permiten distinguir "valor cero real" de "todavia no llego ese canal".
    // id/name identifican el dispositivo; battery/grip/orientation/imuLinearMotion son los canales utiles.
    public string id;
    public string name;
    public int battery;
    public int grip;
    public Quaternion orientation;
    public float imuLinearMotion;
    public int imuPayloadLength;

    [NonSerialized] public bool HasImuData;
    [NonSerialized] public float LastImuUpdateTime;
    [NonSerialized] public bool HasGripData;
    [NonSerialized] public float LastGripUpdateTime;
}

public sealed class BLEManager : MonoBehaviour
{
    // Gestor persistente de BLE. Mantiene conexiones al cambiar de escena y sirve de puente
    // entre los objetos fisicos instrumentados y Unity.
    // La salida publica es connectedDevices + eventos; los demas sistemas no llaman al plugin BLE directo.
    //
    // Flujo simple:
    // 1) Pide permisos Android.
    // 2) Escanea dispositivos esperados.
    // 3) Se conecta y se suscribe a FSR, IMU y bateria.
    // 4) Decodifica bytes en DeviceData para UI y FusionTracker.
    public static BLEManager Instance { get; private set; }

    // --- Dispositivos esperados ---
    // El escaner ignora BLEs que no coincidan con estos nombres o con la tabla MAC.
    [Header("Target Devices (exact BLE advertised names)")]
    public List<string> targetDeviceNames = new()
    {
        // Estos nombres son el contrato entre firmware, fusion y configuracion de prefabs.
        "CubeRed",
        "CubeBlue",
        "MugGreen",
    };

    // --- Resolucion de identidad ---
    // En Quest/Android el nombre anunciado puede quedar cacheado; la MAC corrige esa situacion.
    [Header("MAC to Name Lookup")]
    [Tooltip("Si esta activo, usa macDeviceMap para resolver nombres e ignora el nombre anunciado por BLE.\n" +
             "Hace falta porque Android puede cachear nombres antiguos tras reflashear.")]
    public bool enableMacLookup = true;


    private static readonly Dictionary<string, string> macDeviceMap = new()
    {
        // La MAC actua como identidad fuerte cuando Android conserva nombres BLE obsoletos.
        { "E4:32:5A:81:6E:64", "MugGreen"  },   // antes CubeGreen, reflasheado como MugGreen
        { "FE:13:2A:ED:BC:78", "CubeBlue"  },
        { "D4:91:B7:47:30:BD", "CubeRed"   },
    };

    // --- Contrato GATT con el firmware ---
    // Servicio propio + caracteristicas de IMU/FSR. Bateria usa UUID estandar mas abajo.
    [Header("GATT UUIDs")]
    public string targetServiceUUID = "12345678-1234-5678-1234-56789abcdef0";
    public string imuCharacteristicUUID = "12345678-1234-5678-1234-56789abcdef1";
    public string fsrCharacteristicUUID = "12345678-1234-5678-1234-56789abcdef2";

    // Diccionario runtime: clave = MAC, valor = ultimo estado conocido del dispositivo.
    [Header("Connected Devices")]
    public Dictionary<string, DeviceData> connectedDevices = new();

    // Log de paquetes IMU para diagnosticar firmware/conexion sin tocar FusionTracker.
    [Header("Debug IMU Packets")]
    public bool verboseImuPacketLog = true;
    public float imuPacketLogInterval = 0.5f;

    // Eventos globales: otros sistemas se suscriben para reaccionar a conexion, desconexion o datos nuevos.
    public static event Action<string> OnConnected;
    public static event Action<string> OnDisconnected;
    public static event Action<string> OnDataUpdated;

    // Permisos Android necesarios antes de escanear/conectar BLE en dispositivo real.
    private static readonly string[] RequiredAndroidPermissions = {
        "android.permission.BLUETOOTH_SCAN",
        "android.permission.BLUETOOTH_CONNECT",
        "android.permission.ACCESS_FINE_LOCATION"
    };

    // UUIDs estandar del servicio de bateria BLE.
    private const string BatteryServiceUuid = "180f";
    private const string BatteryCharacteristicUuid = "2a19";

    // --- Estado interno del ciclo BLE ---
    // _initialized evita inicializar dos veces, _isScanning coordina escaneo, _pendingConnections evita duplicados.
    private bool _initialized;
    private bool _isScanning;
    private readonly HashSet<string> _pendingConnections = new();

    // Controla frecuencia de logs por dispositivo para que la consola no se sature.
    private readonly Dictionary<string, float> _lastImuPacketLogTime = new();

    private void Awake()
    {
        // Singleton sencillo: Unity puede cargar otra copia, pero solo una conserva las conexiones.
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;
        DontDestroyOnLoad(gameObject);
    }

    // Arranca la inicializacion BLE tras comprobar permisos Android.
    private void Start() => StartCoroutine(InitWithPermissions());

    private IEnumerator InitWithPermissions()
    {
        // En Android 12+, escaneo y conexion BLE dependen de permisos en runtime.
        // Espera un margen razonable para no inicializar el plugin a medias.
        foreach (string p in RequiredAndroidPermissions)
            if (!Permission.HasUserAuthorizedPermission(p)) Permission.RequestUserPermission(p);

        float timeout = 30f, waited = 0f;
        while (!AllPermissionsGranted() && waited < timeout)
        {
            yield return new WaitForSeconds(0.5f);
            waited += 0.5f;
        }

        if (!AllPermissionsGranted())
        {
            Debug.LogError("[BLE] Permisos no concedidos tras 30s.");
            yield break;
        }
        yield return new WaitForSeconds(0.5f);
        InitializeBluetooth();
    }

    private static bool AllPermissionsGranted()
    {
        // Comprueba todos los permisos antes de tocar el plugin BLE.
        foreach (string p in RequiredAndroidPermissions)
            if (!Permission.HasUserAuthorizedPermission(p)) return false;
        return true;
    }

    private void InitializeBluetooth()
    {
        // Inicializa el adaptador una sola vez y retrasa el primer escaneo para que el plugin asiente.
        if (_initialized) return;
        BluetoothLEHardwareInterface.Initialize(true, false, () =>
        {
            _initialized = true;
            Invoke(nameof(StartAutoScan), 1.0f);
        }, error => Debug.LogError($"[BLE] Error: {error}"));
    }

    private void StartAutoScan()
    {
        // Conecta un dispositivo cada vez porque el plugin BLE falla mas con escaneos/conexiones simultaneas.
        if (_isScanning || connectedDevices.Count >= targetDeviceNames.Count) return;
        _isScanning = true;

        BluetoothLEHardwareInterface.ScanForPeripheralsWithServices(null, (address, name) =>
        {
            if (string.IsNullOrWhiteSpace(name)) return;
            if (connectedDevices.ContainsKey(address) || _pendingConnections.Contains(address)) return;

            bool nameMatch = targetDeviceNames.Contains(name);
            bool macMatch = enableMacLookup && macDeviceMap.ContainsKey(address.ToUpper());

            if (!nameMatch && !macMatch) return;

            Debug.Log($"[BLE] Target found: {name} ({address}) " +
                      $"[nameMatch={nameMatch}, macMatch={macMatch}]");

            _pendingConnections.Add(address);
            BluetoothLEHardwareInterface.StopScan();
            _isScanning = false;
            ConnectToDevice(address, name);
        }, null);
    }

    private void ConnectToDevice(string address, string deviceName)
    {
        // La conexion solo abre canal; los datos llegan tras suscribirse a FSR, IMU y bateria.
        BluetoothLEHardwareInterface.ConnectToPeripheral(address,
            addr => HandlePeripheralConnected(addr, deviceName),
            null, null, HandlePeripheralDisconnected);
    }

    private void HandlePeripheralConnected(string address, string deviceName)
    {
        // El nombre final puede venir de la tabla MAC para evitar nombres cacheados por Android.
        _pendingConnections.Remove(address);
        connectedDevices.Remove(address);

        string finalName = ResolveNameByMac(address, deviceName);
        var device = new DeviceData
        {
            id = address,
            name = finalName,
            battery = 100,
            grip = 0,
            orientation = Quaternion.identity,
            imuLinearMotion = 0f,
            imuPayloadLength = 0,
            HasImuData = false,
            HasGripData = false
        };

        connectedDevices.Add(address, device);

        Debug.Log($"[BLE] Connected: {finalName} ({address}) - Raw OS name: {deviceName}");
        OnConnected?.Invoke(address);
        StartCoroutine(SubscribeToCharacteristics(address));
        Invoke(nameof(StartAutoScan), 1.5f);
    }

    private void HandlePeripheralDisconnected(string address)
    {
        // Al desconectar, reanuda el escaneo para recuperar el dispositivo sin intervencion del usuario.
        _pendingConnections.Remove(address);
        string deviceName = connectedDevices.TryGetValue(address, out DeviceData deviceData)
            ? deviceData.name : address;
        connectedDevices.Remove(address);
        Debug.Log($"[BLE] Disconnected: {deviceName}");
        OnDisconnected?.Invoke(address);
        if (!_isScanning) Invoke(nameof(StartAutoScan), 1.0f);
    }

    private IEnumerator SubscribeToCharacteristics(string address)
    {
        // Pequenas pausas reducen fallos al descubrir varias caracteristicas seguidas en algunos firmwares.
        // Cada yield espera sin bloquear la aplicacion ni congelar la escena.
        yield return new WaitForSeconds(1.5f);
        string serviceUuid = targetServiceUUID.ToLowerInvariant();
        string fsrUuid = fsrCharacteristicUUID.ToLowerInvariant();
        string imuUuid = imuCharacteristicUUID.ToLowerInvariant();

        SubscribeTo(address, serviceUuid, fsrUuid);
        yield return new WaitForSeconds(0.2f);
        SubscribeTo(address, serviceUuid, imuUuid);
        yield return new WaitForSeconds(0.2f);
        SubscribeTo(address, BatteryServiceUuid, BatteryCharacteristicUuid);
    }

    private void SubscribeTo(string addr, string srv, string chr)
    {
        // Cada caracteristica notifica por separado para no mezclar frecuencia IMU con presion FSR.
        BluetoothLEHardwareInterface.SubscribeCharacteristicWithDeviceAddress(addr, srv, chr,
            (_, _) => { },
            (a, c, bytes) => HandleCharacteristicNotification(a, c, bytes));
    }

    private void HandleCharacteristicNotification(string address, string characteristicUUID, byte[] payload)
    {
        // Decodifica cada notificacion segun su UUID y actualiza el paquete publico del dispositivo.
        // Mantiene todos los canales en el mismo DeviceData aunque lleguen con frecuencias distintas.
        // A partir de aqui el resto del proyecto solo lee DeviceData; no necesita conocer bytes ni UUIDs.
        if (!connectedDevices.TryGetValue(address, out DeviceData device) || payload == null || payload.Length == 0) return;

        string uuid = characteristicUUID.ToLowerInvariant();

        if (uuid.Contains(fsrCharacteristicUUID.ToLowerInvariant()))
        {
            DecodeFsrPayload(device, payload);
        }
        else if (uuid.Contains(imuCharacteristicUUID.ToLowerInvariant()) && payload.Length >= 16)
        {
            DecodeImuPayload(device, payload);
        }
        else if (uuid.Contains(BatteryCharacteristicUuid))
        {
            DecodeBatteryPayload(device, payload);
        }

        OnDataUpdated?.Invoke(address);
    }

    private static void DecodeFsrPayload(DeviceData device, byte[] payload)
    {
        // El FSR puede llegar con varios anchos segun firmware; se aceptan int, ushort y byte.
        // FSR = sensor de presion: cuanto mas alto, mas fuerte se esta agarrando.
        device.grip = payload.Length >= 4 ? BitConverter.ToInt32(payload, 0)
                    : payload.Length == 2 ? BitConverter.ToUInt16(payload, 0)
                    : payload[0];
        device.HasGripData = true;
        device.LastGripUpdateTime = Time.realtimeSinceStartup;
    }

    private void DecodeImuPayload(DeviceData device, byte[] payload)
    {
        // Remapea ejes del microcontrolador al sistema de referencia de Unity.
        // IMU = sensor de orientacion/movimiento. Aqui se convierte de bytes a Quaternion.
        device.imuPayloadLength = payload.Length;
        float w = BitConverter.ToSingle(payload, 0);
        float x = BitConverter.ToSingle(payload, 4);
        float y = BitConverter.ToSingle(payload, 8);
        float z = BitConverter.ToSingle(payload, 12);
        device.orientation = new Quaternion(-y, -z, x, w).normalized;
        device.imuLinearMotion = payload.Length >= 20
            ? Mathf.Max(0f, BitConverter.ToSingle(payload, 16))
            : 0f;
        device.HasImuData = true;
        device.LastImuUpdateTime = Time.realtimeSinceStartup;

        LogImuPacket(device);
    }

    private static void DecodeBatteryPayload(DeviceData device, byte[] payload)
    {
        // Bateria estandar BLE: un byte con porcentaje.
        device.battery = payload[0];
    }

    private void LogImuPacket(DeviceData device)
    {
        // Log limitado en frecuencia para revisar paquetes IMU sin saturar la consola.
        if (!verboseImuPacketLog) return;

        float now = Time.realtimeSinceStartup;
        _lastImuPacketLogTime.TryGetValue(device.id, out float lastLog);
        if (now - lastLog < imuPacketLogInterval) return;
        _lastImuPacketLogTime[device.id] = now;

        bool hasLinearMotion = device.imuPayloadLength >= 20;
        string status = hasLinearMotion ? "OK_20B" : "ONLY_16B_NO_ACCEL";
        Debug.Log($"[BLE IMU] {device.name} {status} bytes={device.imuPayloadLength} " +
                  $"linearMotion={device.imuLinearMotion:F4} q={device.orientation.eulerAngles}");
    }

    public DeviceData GetDeviceData(string id)
    {
        // Busca por identificador BLE, que normalmente es la MAC.
        if (string.IsNullOrWhiteSpace(id)) return null;
        connectedDevices.TryGetValue(id, out DeviceData deviceData);
        return deviceData;
    }

    public DeviceData GetDeviceByName(string deviceName)
    {
        // Busca por nombre estable usado por UI, fusion y prefabs.
        if (string.IsNullOrWhiteSpace(deviceName)) return null;

        foreach (var kvp in connectedDevices)
            if (string.Equals(kvp.Value.name, deviceName, StringComparison.OrdinalIgnoreCase))
                return kvp.Value;
        return null;
    }

    private string ResolveNameByMac(string address, string deviceName)
    {
        // Prefiere la tabla MAC; si no existe, asigna un nombre libre para mantener identidades estables.
        if (enableMacLookup && macDeviceMap.TryGetValue(address.ToUpper(), out string mapped))
        {
            Debug.Log($"[BLE] MAC {address} -> {mapped} (OS decia: {deviceName})");
            return mapped;
        }

        if (targetDeviceNames.Contains(deviceName) && GetDeviceByName(deviceName) == null)
            return deviceName;

        foreach (string target in targetDeviceNames)
            if (GetDeviceByName(target) == null) return target;
        return deviceName;
    }

    private void OnApplicationQuit()
    {
        // Cierra conexiones explicitamente para que Android no deje el adaptador BLE en estado viejo.
        if (_isScanning) BluetoothLEHardwareInterface.StopScan();
        foreach (string address in new List<string>(connectedDevices.Keys))
            BluetoothLEHardwareInterface.DisconnectPeripheral(address, null);
        BluetoothLEHardwareInterface.DeInitialize(() => { });
    }

    // Usa el mismo cierre si Unity destruye el objeto antes de salir de la aplicacion.
    private void OnDestroy() => OnApplicationQuit();
}
