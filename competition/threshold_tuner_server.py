# Image Transfer - As The Remote Device
#
# This script is meant to talk to the "image_transfer_jpg_streaming_as_the_controller_device.py" on your computer.
#
# This script shows off how to transfer the frame buffer to your computer as a jpeg image.

import network, omv, sensor, math, image
import rpc
from pyb import Pin

#def threshold_tuner_server():
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 500)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 500)
#sensor.set_auto_gain(False, gain_db=0 ) # 9.172951)
#sensor.skip_frames(time = 500)
#sensor.set_auto_exposure(False, exposure_us=2000) #6576)
#sensor.skip_frames(time = 500)
sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0, -3.0, 3.0))

img = sensor.snapshot()

pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
ala = 0
#while(ala==0):
    #if (pin2.value()== 0):   # нажатие на кнопку 2 на голове
        #ala = 1
        #print("нажато")

# Turn off the frame buffer connection to the IDE from the OpenMV Cam side.
#
# This needs to be done when manually compressing jpeg images at higher quality
# so that the OpenMV Cam does not try to stream them to the IDE using a fall back
# mechanism if the JPEG image is too large to fit in the IDE JPEG frame buffer on the OpenMV Cam.

omv.disable_fb(True)


# The RPC library above is installed on your OpenMV Cam and provides mutliple classes for
# allowing your OpenMV Cam to be controlled over USB or WIFI.

################################################################
# Choose the interface you wish to control your OpenMV Cam over.
################################################################

# Uncomment the below line to setup your OpenMV Cam for control over a USB VCP.
#
interface = rpc.rpc_usb_vcp_slave()

# Uncomment the below line to setup your OpenMV Cam for control over WiFi.
#
# * ssid - WiFi network to connect to.
# * ssid_key - WiFi network password.
# * ssid_security - WiFi security.
# * port - Port to route traffic to.
# * mode - Regular or access-point mode.
# * static_ip - If not None then a tuple of the (IP Address, Subnet Mask, Gateway, DNS Address)
#
#interface = rpc.rpc_wifi_slave(ssid="FED1",
                            #ssid_key="7684067a",
                            #ssid_security=network.WINC.WPA_PSK,
                            #port=0x1DBA,
                            #mode=network.WINC.MODE_STA,
                            #static_ip=None)

################################################################
# Call Backs
################################################################

# This is called repeatedly by interface.stream_writer().
def stream_generator_cb():
    return sensor.snapshot().compress(quality=90).bytearray()

# Transmits a stream of bytes()'s generated by stream_generator_cb to the master device.
def jpeg_image_stream_cb():
    interface.stream_writer(stream_generator_cb)

# When called sets the pixformat and framesize, and then schedules
# frame streaming to start after the RPC call finishes.
#
# data is a pixformat string and framesize string.
def find_orange_ball_on_green_field(img, data_dict):
    blobs = img.find_blobs([data_dict['orange ball']['th']],
                            pixels_threshold=data_dict['orange ball']['pixel'],
                            area_threshold=data_dict['orange ball']['area'],
                            merge=True, margin=10)
    if (len (blobs) == 1):
        blob = blobs [0]
        x , y , w , h = blob.rect()  # ball blob
        x1 = x + w                      # x1, y1, w1, h1-right rectangle
        y1 = y
        if x1 + w <= 320:
            w1 = w
        else:
            w1 = 320 - x1
        if x1 == 320:
            w1 = 1
            x1 = 319
        if y1 + 2 * h <= 240:
            h1 = 2 * h
        else:
            h1 = 240 - y1
        if y1 + h == 240:
            h1 = h
        y2 = y                         # x2, y2, w2, h2 - left rectangle
        if x - w > 0:
            x2 = x - w
            w2 = w
        else:
            x2 = 0
            w2 = x1 - x2
        if x1 == 0:
            x2 = 0
            w2 = 1
        y2 = y1
        h2 = h1
        x3 = x                          # x3, y3, w3, h3 - bottom rectangle
        y3 = y + h - 1
        w3 = w
        h3 = h1 - h + 1
        blob_p = []                     # right blobs
        blob_l = []                     # left blobs
        blob_n = []                     # bottom blobs
        blob_p = img.find_blobs([data_dict['green field']['th']],roi = [x1 , y1 , w1 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_l = img.find_blobs([data_dict['green field']['th']],roi = [x2 , y1 , w2 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_n = img.find_blobs([data_dict['green field']['th']],roi = [x3 , y3 , w3 , h3], pixels_threshold=7, area_threshold=7, merge=True)
        if len(blob_p) > 0 or len( blob_l ) > 0  or len( blob_n ) > 0:
            img.draw_rectangle(blob.rect(), color = (255, 0, 0))

def detect_Post_In_image(img, data_dict, post_color):
    post_thresholds =  [data_dict[post_color]['th']]
    for blob in img.find_blobs(post_thresholds, pixels_threshold = data_dict[post_color]['pixel'],
                              area_threshold = data_dict[post_color]['area'], merge=True):
        blob_Is_Post = False
        if blob.y() + blob.h() > 235 : continue         # blob connected to bottom of picture. No opportunity to recognize data
        else:
            if blob.w() > 314: continue         # blob connected to both sides of picture. No opportunity to recognize data
            for y in range (blob.y() + blob.h(), blob.y() + blob.h() + 5, 1 ):
                for x in range (blob.x(), blob.x() + blob.w(), 1 ):
                    a= image.rgb_to_lab(img.get_pixel(x , y))
                    is_green = (data_dict['green field']['th'][0] < a[0] < data_dict['green field']['th'][1]) and \
                               (data_dict['green field']['th'][2] < a[1] < data_dict['green field']['th'][3]) and \
                               (data_dict['green field']['th'][4] < a[2] < data_dict['green field']['th'][5])
                    is_white = (data_dict['white marking']['th'][0] < a[0] < data_dict['white marking']['th'][1]) and \
                               (data_dict['white marking']['th'][2] < a[1] < data_dict['white marking']['th'][3]) and \
                               (data_dict['white marking']['th'][4] < a[2] < data_dict['white marking']['th'][5])
                    if is_green == True or is_white == True : blob_Is_Post = True
                    if blob_Is_Post == True: break
                if blob_Is_Post == True: break
            if blob_Is_Post == True:
                img.draw_rectangle(blob.rect(), color = (255, 0, 0))

def find_all_blobs(img, data_dict):
    current = data_dict['current']
    for blob in img.find_blobs([data_dict[current]["th"]],
                    pixels_threshold=data_dict[current]["pixel"],
                    area_threshold=data_dict[current]["area"], merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255,0,0))
            img.draw_line(blob.major_axis_line(), color=(0,255,0))
            img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

thresholds = None
even_frame = False
def jpeg_image_stream(data):
    pixformat, framesize = bytes(data).decode().split(",")
    sensor.set_pixformat(eval(pixformat))
    sensor.set_framesize(eval(framesize))
    interface.schedule_callback(jpeg_image_stream_cb)
    return bytes()

def binary_stream_generator_cb():
    global thresholds
    global even_frame
    data_dict = eval(thresholds)
    current = data_dict['current']
    img = sensor.snapshot()
    if even_frame:
        img.binary([data_dict[current]["th"]])
        even_frame = False
    else:
        even_frame = True
        if data_dict["blob"] == 2:
            find_orange_ball_on_green_field(img, data_dict)
        elif data_dict["blob"] == 3:
            detect_Post_In_image(img, data_dict, "blue posts")
        elif data_dict["blob"] == 4:
            detect_Post_In_image(img, data_dict, "yellow posts")
        elif data_dict["blob"] == 5:
            detect_Post_In_image(img, data_dict, "white posts")
        elif data_dict["blob"] > 0:
            find_all_blobs(img, data_dict)
    return img.compress(quality=90).bytearray()

def jpeg_image_binary_stream_cb():
    interface.stream_writer(binary_stream_generator_cb)

def jpeg_image_binary_stream(data):
    global thresholds
    thresholds = bytes(data).decode()
    print(thresholds)
    interface.schedule_callback(jpeg_image_binary_stream_cb)
    return bytes()

def jpeg_snapshot(data):
    img = sensor.snapshot()
    img1 = img.compress(quality=90)
    return img.bytearray()

def binary_snapshot(data):
    img = sensor.snapshot()
    thresholds = bytes(data).decode()  #(0, 100, -65, -37, 10, 64)
    print(thresholds)
    img.binary([eval(thresholds)])
    return img.compress(quality=90).bytearray()

# Register call backs.

interface.register_callback(jpeg_image_stream)
interface.register_callback(jpeg_image_binary_stream)
interface.register_callback(jpeg_snapshot)
interface.register_callback(binary_snapshot)

# Once all call backs have been registered we can start
# processing remote events. interface.loop() does not return.

interface.loop()

#if __name__=="__main__":
    #threshold_tuner_server()
