import socket
import sys

#Red simulación
host = "127.0.0.1"
#Red robot
#host = "192.168.125.1"
puerto = 5000


def cliente_tcp():
    # Solicitar datos de conexión
    # host = input("Introduce la IP o hostname del servidor: ").strip()
    # try:
    #     puerto = int(input("Introduce el puerto: ").strip())
    # except ValueError:
    #     print("Error: El puerto debe ser un número entero.")
    #     return

    # Crear el socket TCP/IP
    cliente = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Timeout opcional para evitar bloqueos indefinidos al recibir
    cliente.settimeout(100.0)

    try:
        print(f"Conectando a {host}:{puerto}...")
        cliente.connect((host, puerto))
        print("Conexión establecida con éxito. Escribe 'exit' para salir.\n")

        while True:
            mensaje = input("Mensaje a enviar > ")
            mensaje += "\n"

            # Comando especial para salir
            if mensaje.strip().lower() == "exit":
                print("Cerrando sesión...")
                break

            if not mensaje:
                continue

            # Enviar mensaje codificado en bytes
            cliente.sendall(mensaje.encode("utf-8"))

            # Esperar e imprimir la respuesta
            try:
                respuesta = cliente.recv(4096)
                if not respuesta:
                    print("El servidor cerró la conexión.")
                    break
                print(f"Respuesta recibida: {respuesta.decode('utf-8')}\n")
            except socket.timeout:
                print("Tiempo de espera agotado: No se recibió respuesta del servidor.\n")

    except ConnectionRefusedError:
        print(f"Error: No se pudo conectar a {host}:{puerto}. Conexión rechazada.")
    except ConnectionResetError:
        print("El host remoto cerró/resetó la conexión TCP.")
    except socket.gaierror:
        print(f"Error: No se pudo resolver el host '{host}'.")
    except KeyboardInterrupt:
        print("\nInterrupción detectada por teclado.")
    except Exception as e:
        print(f"Ocurrió un error inesperado: {e}")
    finally:
        cliente.close()
        print("Socket cerrado. Programa finalizado.")

if __name__ == "__main__":
    cliente_tcp()