import socket
import struct

MCAST_GRP = '225.0.0.1'
MCAST_PORT = 31122
IS_ALL_GROUPS = False
interfaceIP = struct.unpack(">L", socket.inet_aton('192.168.1.30'))[0]
print(interfaceIP)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32) 
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

if IS_ALL_GROUPS:
    # on this port, receives ALL multicast groups
    sock.bind(('', MCAST_PORT))
else:
    # on this port, listen ONLY to MCAST_GRP
    sock.bind((MCAST_GRP, MCAST_PORT))
# mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), interfaceIP)

# sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# while True:
#   print sock.recv(10240)

# host = socket.gethostbyname(socket.gethostname())
host = '192.168.1.30'
print('host: ' + host)
sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(host))
sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, 
               socket.inet_aton(MCAST_GRP) + socket.inet_aton(host))

while 1:
	try:
		data, addr = sock.recvfrom(1154)
		print("New Packet!")
		print("Address: " + str(addr))
		print(data)
	except socket.error, e:
		print 'Expection'
		hexdata = binascii.hexlify(data)
		print 'Data = %s' % hexdata
