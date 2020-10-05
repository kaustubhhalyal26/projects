import pyttsx3 #pip install pyttsx3
import datetime
engine = pyttsx3.init()#main engine to respond in voice 
''' voice setting function '''
def voicegenerator():
    global engine 
    voices = engine.getProperty('voices')
    # print(voices[1].id)
    engine.setProperty('voice', voices[1].id)

''' speak function '''
def speak(audio):
    global engine
    engine.say(audio)
    engine.runAndWait()

''' greeting function for starting convo '''
def greet():
    hour = int(datetime.datetime.now().hour)
    speak("Hello")
    if hour>=0 and hour<12:
        speak("Good Morning!")

    elif hour>=12 and hour<18:
        speak("Good Afternoon!")   

    else:
        speak("Good Evening!")  
