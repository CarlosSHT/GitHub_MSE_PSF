import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
 

header = { "head": b"head", "id": 0, "N": 128, "fs": 10000, "tail":b"tail" }
fig    = plt.figure ( 1 )

adcAxe = fig.add_subplot ( 2,1,1                  )
adcLn, = plt.plot        ( [],[],'r-',linewidth=4 )
adcAxe.grid              ( True                   )
adcAxe.set_ylim          ( -1.65 ,1.65            )

fftAxe = fig.add_subplot ( 2,1,2                  )
fftLn, = plt.plot        ( [],[],'b-',linewidth=4 )
fftAxe.grid              ( True                   )
fftAxe.set_ylim          ( 0 ,0.25                )


localIP     = "192.168.60.125"
# localIP     = "192.168.5.229"
localPort   = 61454
bufferSize  = 1400

# msgFromServer       = "Hello UDP Client"
# bytesToSend         = str.encode(msgFromServer)



# # Listen for incoming datagrams
# while(True):

#     print("Nuevo entrada UDP")
#     bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
#     print("Esperandoando")
#     message = bytesAddressPair[0]

#     address = bytesAddressPair[1]

#     clientMsg = "Message from Client:{}".format(message)
#     clientIP  = "Client IP Address:{}".format(address)
    
#     print(clientMsg)
#     print(clientIP)

#     # Sending a reply to client

#     # UDPServerSocket.sendto(bytesToSend, address)



def init():
    # ax.set_xlim(0, 2*np.pi)
    # ax.set_ylim(-1, 1)
    return adcLn,

def validate_header(msg, h):
    ret = False
    idx_head=msg.find(b"head")
    idx_tail=msg.find(b"tail")

    h["id"] = int.from_bytes(msg[len(h["head"]):len(h["head"])+4], byteorder='little', signed=False)
    h["N" ] = int.from_bytes(msg[len(h["head"])+4:len(h["head"])+6], byteorder='little', signed=False)
    h["fs"] = int.from_bytes(msg[len(h["head"])+6:len(h["head"])+8], byteorder='little', signed=False)

    print( "id :  {}".format(h["id"]))
    print( "N :  {}".format(h["N"]))
    print( "fs :  {}".format(h["fs"]))

    data_len=len(msg)-(len(h["head"])+len(h["tail"])+8)
    # print( "data_len :  {}".format(data_len))

    if idx_head == 0 and idx_tail == 12 and data_len/2 == h["N"]:
        ret = True
    return  h["id"],h["N"],h["fs"], ret

 


def readSamples(adc, N, msg):
    offset=16
    l=np.arange(0,N,1)*2
    
    for i in range (N):
        adc[i]= ((int.from_bytes(msg[16++l[i]:16++l[i]+2], byteorder='little', signed=True))/512)*1.65


# def readSamples(adc,N,trigger=False,th=0):
#     state="waitLow" if trigger else "sampling"
#     i=0
#     for t in range(N):
#         sample = readInt4File(streamFile,sign = True)/512*1.65
#         state,nextI= {
#                 "waitLow" : lambda sample,i: ("waitHigh",0) if sample<th else ("waitLow" ,0),
#                 "waitHigh": lambda sample,i: ("sampling",0) if sample>th else ("waitHigh",0),
#                 "sampling": lambda sample,i: ("sampling",i+1)
#                 }[state](sample,i)
#         adc[i]=sample
#         i=nextI

def update(frame):
    
    global header

    # print("Nuevo entrada UDP")
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    # print("Paquete recibido")
    message = bytesAddressPair[0]

    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    # print(clientMsg)
    # print(clientIP)
    # print(message.find(b"head"))
    # print(message.find(b"tail"))

    id,N,fs,ret=validate_header(message,header)

    adc   = np.zeros(N)
    time  = np.arange(0,N/fs,1/fs)

    # readSamples(adc,N,False,-1.3)
    readSamples(adc, N,message)

    adcAxe.set_xlim ( 0    ,N/fs )
    adcLn.set_data  ( time ,adc  )

    fft=np.abs ( 1/N*np.fft.fft(adc ))**2
    fftAxe.set_ylim ( 0 ,np.max(fft)+0.05)
    fftAxe.set_xlim ( 0 ,fs/2 )
    fftLn.set_data ( (fs/N )*fs*time ,fft)
    return adcLn, fftLn


if __name__ == '__main__':
    # Create a datagram socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    # Bind to address and ip
    UDPServerSocket.bind((localIP, localPort))

    # print("UDP server up and listening")


    # ani = FuncAnimation(fig,func= update, frames=np.linspace(0, 2*np.pi, 128), init_func=init, blit=True, interval=1)
    ani = FuncAnimation(fig,func= update, frames=1000, init_func=None, blit=True, interval=1,repeat=True)
    # ani = FuncAnimation(fig,update,10000,init_func=None,blit=True,interval=1,repeat=True)
    plt.show()
