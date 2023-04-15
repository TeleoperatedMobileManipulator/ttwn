import multiprocessing
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robotarm_interfaces.msg import Sensordeg
from robotarm_interfaces.msg import Inversedegrees
import firebase_admin
from firebase_admin import credentials, firestore
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, MediaStreamTrack
#from aiortc.RTCConfiguration import RTCConfiguration
from aiortc.rtcrtpsender import RTCRtpSender
import os
from aiortc.contrib.media import MediaPlayer, MediaRelay
import logging
import cv2
import numpy as np
import math
from av import VideoFrame
import argparse





#logging.basicConfig(level=logging.DEBUG)



# Config
# Load the service account key file
cred = credentials.Certificate('/home/tawanpc/ros2ws/src/robotarm_pkg/robotarm_pkg/serviceAccoutKey.json')

pc = None
firestore_ref = None
offer_chan = None
logger = logging.getLogger("pc")
state_chan = None


webcam = None
options = {"framerate": "30", "video_size": "640x480"}
relay = None
video = None
video_sender = None
video_track = None
video_stream = None

data_webrtc = None
share_J1 = None
share_J2 = None
share_J3 = None
share_J4 = None
share_X = None
share_Y = None
share_Z = None
time_start = None

check_icecandidate = 'closed'


class videoSubscriber(Node):

  def __init__(self):
    super().__init__('videocommu')
    self.publisher_ = self.create_publisher(String, 'webint_top', 10)
    self.subscriber_ = self.create_subscription(Sensordeg, "sensordegre_top", self.callback_robot, 10)
    self.subscriber_ = self.create_subscription(Inversedegrees, "realxyz_top", self.callback_realxyz, 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.receive_datartc)
    self.i = 0

  def callback_robot(self,msg):
    global share_J1
    global share_J2
    global share_J3
    global share_J4
    share_J1.value = msg.sendegre1
    share_J2.value = msg.sendegre2
    share_J3.value = msg.sendegre3
    share_J4.value = msg.sendegre4

  def callback_realxyz(self,msg):
    global share_X
    global share_Y
    global share_Z

    share_X.value = msg.inversej1
    share_Y.value = msg.inversej2
    share_Z.value = msg.inversej3

  def receive_datartc(self):
    global data_webrtc
    
    msg = String()
    msg.data = '%s' % data_webrtc.value
    self.publisher_.publish(msg)
    #self.get_logger().info('Publishing: "%s"' % msg.data)
   

def channel_log(channel, t, message):
    logger.info("channel(%s) %s %s" % (channel.label, t, message))
    print("channel(%s) %s %s" % (channel.label, t, message))

def channel_send(channel, message):
    channel_log(channel, ">", message)
    channel.send(message)

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

# Subclass of MediaStreamFrack

def create_local_tracks():
    global relay
    global options
    global video
    global video_stream
    relay = MediaRelay()
    '''
    try:
      video_stream = MediaPlayer("/dev/video0", format="v4l2", options=options)
      #video = relay.subscribe(video_stream.video)
      video = video_stream.video
    except :
      video = None
    '''
    video_stream = MediaPlayer("/dev/video0", format="v4l2", options=options)
    #video = relay.subscribe(video_stream.video)
    video = video_stream.video
    print(video)
    return video

    
def current_stamp():
    global time_start

    if time_start is None:
        time_start = time.time()
        return 0
    else:
        return int((time.time() - time_start) * 1000000)  

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )
 
class SimpleVideoStreamTrack(VideoStreamTrack):
    """
    A video track thats returns camera track with annotated detected objects.
    """
    def __init__(self, conf_thres=0.7, iou_thres=0.5):
        super().__init__()  # don't forget this!
        self.conf_threshold = conf_thres
        self.iou_threshold = iou_thres
        global video
        for i in range(10):
          cap = cv2.VideoCapture(i)
          if cap.read()[0]:
              print(f"Camera {i} is available")
          cap.release()
        try:
          video = cv2.VideoCapture(0)
        except Exception as e:
          print(f"Failed to open camera: {e}")
          pass
        
        if not video.isOpened():
          print("Cannot open camera")
          video = None
        else:
          print("Camera is ready")
          self.video = video

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        _, frame = self.video.read()
        frame = VideoFrame.from_ndarray(frame, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame

def current_stamp():
    global time_start

    if time_start is None:
        time_start = time.time()
        return 0
    else:
        return int((time.time() - time_start) * 1000000)  

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

#function check data if exist  
def checkandgetdata(collec,doc):
  col_str = str(collec)
  doc_str = str(doc)
  data = firestore_ref.collection(col_str).document(doc_str).get()

  # Check if the document exists
  if data.exists:
    # Document exists, get the data
    print('Receive signalling: complete ')
  else:
    # Document does not exist, loop and wait for it to be created
    while True:
      data = firestore_ref.collection(col_str).document(doc_str).get()
      print('wait : ', doc_str)
      if data.exists:
        print('Receive signalling: ',data.to_dict())
        break
      time.sleep(1)
  return data.to_dict()

def handle_offer(doc_snapshot, changes, read_time):
  global offer_chan 
  #offer_chan = checkandgetdata(collec='signalling',doc='offer')
  offer_chan = firestore_ref.collection('signalling').document('offer').get().to_dict()

async def handle_state(doc_snapshot, changes, read_time):
  global state_chan 
  global pc
  #state_chan = checkandgetdata(collec='signalling',doc='status')
  state_chan = firestore_ref.collection('signalling').document('status').get().to_dict()
  print('state_chan[status]', state_chan['status'])
  if state_chan['status'] == 'disconnect':   
    global video_track
    global video_sender
    global video_stream
    global check_icecandidate 
    await asyncio.sleep(0.25)
    try:
      #video_track = None    
      check_icecandidate = 'closed'
    except RuntimeError:
      pass


    '''
    try:
      #video_track = None    
      if video_sender!= None:
        #video_stream.video.stop()
        await video_sender.stop()
        await asyncio.sleep(2)
      print('pc is closed')
      await pc.close()
      check_icecandidate = 'close'
    except RuntimeError:
      pass
    await pc.close()  
    '''
    
  
    if pc.iceConnectionState == 'closed': #restart webrtc
      global data_webrtc
      global relay
      data_webrtc.value = b'E'
      global video
      video.release()
      pc = RTCPeerConnection()
      #asyncio.sleep(0.25)
      #relay = None
      #video = None


async def handle_changes_sdp():
    while True:
      global offer_chan
      global pc
      print('offer_chan:',offer_chan)  
      print('iceConnectionState: ',pc.iceConnectionState)
      await asyncio.sleep(0.3)
      if pc.iceConnectionState == 'new':
        #print('state_chan[status]', state_chan['status'])
        @pc.on("datachannel")
        def on_datachannel(channel):
          channel_log(channel, "-", "created by remote party")

          @channel.on("message")
          def on_message(message):
              channel_log(channel, "<", message)
              char_message = list(message)
              print(char_message)
              print(char_message[0])
              print(type(char_message[0]))
              global data_webrtc
              data_webrtc.value= message.encode('ascii')

        @pc.on("connectionstatechange")
        def on_connectionstatechange(x=None):
            logger.info("connectionstatechange: %s" % pc.connectionState)

        @pc.on("signalingstatechange")
        def on_signalingstatechange(x=None):
            logger.info("signalingstatechange: %s" % pc.signalingState)

        @pc.on("icegatherstatechange")
        def on_signalingstatechange(x=None):
            logger.info("pc.iceGatheringState: %s" % pc.iceGatheringState)

        @pc.on("iceconnectionstatechange")
        def on_signalingstatechange(x=None):
            logger.info("pc.iceGatheringState: %s" % pc.iceGatheringState)

        '''
        @pc.on("track")
        def on_track(track):
            logger.info("Track %s received", track.kind) 
            if track.kind == "video":
                pc.addTrack(
                    VideoTransformTrack(
                        relay.subscribe(track), transform=params["video_transform"]
                    )
                )
        '''

        await asyncio.sleep(1) 
        print('handle iceConnectionState: ',pc.iceConnectionState)
            # Add logic to handle changes to the document here
            #print(data)
        if offer_chan:
          #send video
          global relay  
          global video_sender
          
          try:
            pc.addTrack(SimpleVideoStreamTrack())
          except Exception as e:
            print(f"Failed to add track: {e}")
            pass

          #create channel
          channel = pc.createDataChannel("sensor")
          channel_log(channel, "-", "created by local party")

          # Convert the data in aiortc
          offer_sdp = offer_chan['sdp']
          offer_type = offer_chan['type']
          offer = RTCSessionDescription(sdp=offer_sdp, type=offer_type)
          await pc.setRemoteDescription(offer)

          # Create answer
          answer_pc =await pc.createAnswer()
          answer = RTCSessionDescription(sdp=answer_pc.sdp, type=answer_pc.type)
          await pc.setLocalDescription(answer)
          #await asyncio.sleep(2)
          Remote_description = pc.localDescription
          if(Remote_description):        
              # Send answer to firebase
              answer_dict = {
                'sdp': Remote_description.sdp,
                'type': Remote_description.type
                }
              firestore_ref.collection('signalling').document('answer').set(answer_dict)
              print('send answer: succeesful')

          else:
              print('setLocalDescription: faild')
              print('send answer: faild')
          await asyncio.sleep(1)

          global check_icecandidate
          check_icecandidate = 'completed'
          while True:
              await asyncio.sleep(0.6)
              if pc.iceConnectionState == 'completed':
                print('status : connection is complete')  
                global share_J1
                global share_J2
                global share_J3
                global share_J4
                global share_X
                global share_Y 
                global share_Z 
                sen1 = round((share_J1.value),1)
                sen2 = round((share_J2.value),1)
                sen3 = round((share_J3.value),1)
                sen4 = round((share_J4.value),1)
                sen5 = round((share_X.value),1)
                sen6 = round((share_Y.value),1)
                sen7 = round((share_Z.value),1)
                datatoSend = str(sen1) + ' ' + str(sen2) + ' ' + str(sen3) + ' ' + str(sen4) + ' ' + str(time.time()*1000) + ' ' + str(sen5) + ' ' + str(sen6) + ' ' + str(sen7) 
                #print('data to send : ', datatoSend) 
                channel_send(channel,datatoSend)
                if check_icecandidate == 'closed':
                  await asyncio.sleep(0.2)
                  #video_stream.video.stop()
                  #await video_sender.stop()
                  global video
                  if video != None:
                    video.release()
                  
                  video = None
                  await pc.close()
              elif pc.iceConnectionState == 'new':
                offer_chan = None
                break
              else:
                print('[status] iceConnectionState: ', pc.iceConnectionState)
                pc = RTCPeerConnection()
                
      elif pc.iceConnectionState == 'closed':
         pass
      elif pc.iceConnectionState == 'faild':
         pc = RTCPeerConnection()
      elif pc.iceConnectionState == 'completed':
         pc.close()
         print('-------close-------')

     

def ros2_task(args=None):
  rclpy.init(args=args)
  rclpy.spin(videoSubscriber())
  rclpy.shutdown()

def commuint_task():
  #Initialize the app with the service account
  firebase_admin.initialize_app(cred)

  #Get a referance to the firestore
  global firestore_ref
  firestore_ref = firebase_admin.firestore.client(app=None) 

  #Create a peer connection
  global pc
  pc = RTCPeerConnection(
    #configuration=RTCConfiguration(iceServers=[RTCIceServer(
    #urls= "turn:relay.metered.ca:80",
    #username= "02dbe728ee6caa98946ba44c",
    #credential= "6atIXg/6oa2P9Wps",
    #)])
    )
  
  
  @pc.on("connectionstatechange")
  def on_connectionstatechange(x=None):
      logger.info("connectionstatechange: %s" % pc.connectionState)

  @pc.on("signalingstatechange")
  def on_signalingstatechange(x=None):
      logger.info("signalingstatechange: %s" % pc.signalingState)

  @pc.on("icegatherstatechange")
  def on_signalingstatechange(x=None):
      logger.info("pc.iceGatheringState: %s" % pc.iceGatheringState)

  @pc.on("iceconnectionstatechange")
  def on_signalingstatechange(x=None):
      logger.info("pc.iceGatheringState: %s" % pc.iceGatheringState)
  
  offer_firebase = firestore_ref.collection('signalling').document('offer')
  docs = offer_firebase.on_snapshot(lambda doc_snapshot, changes, read_time: handle_offer(doc_snapshot, changes, read_time))
  state_con = firestore_ref.collection('signalling').document('status')
  docs = state_con.on_snapshot(lambda doc_snapshot, changes, read_time: asyncio.run(handle_state(doc_snapshot, changes, read_time)))
  

  asyncio.run(handle_changes_sdp())
  

   

def main(args=None):
  global data_webrtc
  global share_J1
  global share_J2
  global share_J3
  global share_J4
  global share_X
  global share_Y
  global share_Z
  data_webrtc = multiprocessing.Value('c',b'E')
  share_J1 = multiprocessing.Value('f',0.0)
  share_J2 = multiprocessing.Value('f',0.0)
  share_J3 = multiprocessing.Value('f',0.0)
  share_J4 = multiprocessing.Value('f',0.0)
  share_X = multiprocessing.Value('f',0.0)
  share_Y = multiprocessing.Value('f',0.0)
  share_Z = multiprocessing.Value('f',0.0)
  p1 = multiprocessing.Process(target = commuint_task)
  p2 = multiprocessing.Process(target = ros2_task)
  
  p1.start()
  p2.start()

  p1.join()
  p2.join()
  

if __name__ == '__main__':
  main()

