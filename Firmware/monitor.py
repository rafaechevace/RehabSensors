import asyncio
import struct
from bleak import BleakScanner, BleakClient

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

DEVICE_NAME = "RehabIMU"

# UUIDs (Deben estar en minúsculas para comparar fácil)
UUID_IMU_CHAR = "12345678-1234-5678-1234-56789abcdef1"
UUID_FSR_CHAR = "12345678-1234-5678-1234-56789abcdef2"

# ==============================================================================
# DECODIFICADORES
# ==============================================================================

def decode_imu(data):
    if len(data) != 16:
        return
    q0, q1, q2, q3 = struct.unpack('<4f', data)
    print(f"[IMU] Q: {q0:.2f}, {q1:.2f}, {q2:.2f}, {q3:.2f}")

def decode_fsr(data):
    if len(data) != 4:
        return
    # Decodificamos el entero (Little Endian)
    val = struct.unpack('<i', data)[0]
    
    # Visualización gráfica
    # Si el valor es > 1500 (tu umbral de GRIP), mostramos ALERTA
    estado = "REPOSO"
    if val > 1500:
        estado = "AGARRE DETECTADO ✊"
    elif val > 1200:
        estado = "SOLTANDO ✋"
        
    bar_len = int(val / 50) # Ajuste visual
    bar = "█" * bar_len
    print(f"[FSR] {val:04d} {bar} ({estado})")

# ==============================================================================
# CALLBACKS DE NOTIFICACIÓN (CORREGIDO)
# ==============================================================================

def notification_handler(sender, data):
    # CORRECCIÓN: 'sender' es un objeto complejo. Accedemos a su propiedad .uuid
    # Si por alguna razón es un string, lo usamos directo.
    if hasattr(sender, 'uuid'):
        sender_uuid = sender.uuid.lower()
    else:
        sender_uuid = str(sender).lower()
    
    # Comparamos
    if sender_uuid == UUID_IMU_CHAR.lower():
        decode_imu(data)
    elif sender_uuid == UUID_FSR_CHAR.lower():
        decode_fsr(data)
    else:
        # Debug solo si llega algo raro
        print(f"[RAW] {sender_uuid}: {data.hex()}")

# ==============================================================================
# MAIN
# ==============================================================================

async def main():
    print(f"🔎 Buscando '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    if device is None:
        print(f"❌ No se encontró '{DEVICE_NAME}'.")
        return

    print(f"✅ Conectando a {device.address}...")

    async with BleakClient(device) as client:
        print(f"🔗 Conectado.")
        
        # Suscribirse
        await client.start_notify(UUID_IMU_CHAR, notification_handler)
        await client.start_notify(UUID_FSR_CHAR, notification_handler)
        
        print("\n--- MONITORIZANDO (Aprieta el sensor para ver cambios) ---\n")

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nDesconectando...")
            await client.stop_notify(UUID_IMU_CHAR)
            await client.stop_notify(UUID_FSR_CHAR)

if __name__ == "__main__":
    asyncio.run(main())