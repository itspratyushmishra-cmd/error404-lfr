import socket

# Configuration
HOST = '0.0.0.0'  # Listens on all available network interfaces
PORT = 65432      # Arbitrary non-privileged port

print(f"Starting server on port {PORT}...")

# Create a TCP/IP socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    
    # Find your computer's local IP to enter into the Arduino code
    local_ip = socket.gethostbyname(socket.gethostname())
    print(f"Server is listening. Enter this IP into your Arduino code: {local_ip}")
    print("Waiting for Arduino to connect...")
    
    # Accept the connection
    conn, addr = s.accept()
    with conn:
        print(f"Connected by Arduino at {addr}")
        
        while True:
            # Receive data from Arduino (up to 1024 bytes)
            data = conn.recv(1024)
            if not data:
                break # Arduino disconnected
                
            # Decode and print the received message
            message = data.decode('utf-8').strip()
            print(f"Arduino says: {message}")
            
            # Send a response back to the Arduino
            response = "Message received by Python!\n"
            # conn.sendall(response.encode('utf-8'))