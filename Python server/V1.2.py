import socket

HOST = '0.0.0.0'  
PORT = 65432      

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    
    print(f"Listening on port {PORT}...")
    
    while True: # Outer loop keeps server running even if Arduino turns off
        try:
            conn, addr = s.accept()
            with conn:
                print(f"Arduino connected from {addr}")
                
                while True:
                    data = conn.recv(1024)
                    
                    # If recv() returns empty data, the Arduino disconnected or lost power
                    if not data:
                        print("Arduino disconnected. Waiting for reconnection...\n")
                        break 
                    
                    # Decode and remove invisible characters (\r, \n)
                    message = data.decode('utf-8').strip()
                    
                    # Only print if the string is not empty
                    if message:
                        print(f"{message}")
                        
        except (ConnectionResetError, BrokenPipeError):
            print("Connection abruptly lost. Waiting for reconnection...\n")
        except KeyboardInterrupt:
            print("\nServer stopped.")
            break