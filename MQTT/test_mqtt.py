import paho.mqtt.client as mqtt #import the client1
import time

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

#broker_address="192.168.1.184"
broker_address="broker.mqttdashboard.com" #use external broker
client = mqtt.Client("P1") #create new instance
client.on_message=on_message #attach function to callback
client.connect(broker_address, port=1883) #connect to broker
topic = "darkone03/main-light"
client.loop_start() #start the loop
print("Subscribing to topic",topic)
client.subscribe(topic, qos=2)
client.publish(topic,payload="OFF", qos=2)
time.sleep(4) # wait
client.loop_stop() #stop the loop