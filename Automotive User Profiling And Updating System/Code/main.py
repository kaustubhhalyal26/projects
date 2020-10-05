from face_dataset import *
from face_training import *
from facerecognition_prog import *
from file_handling import *
from voice_over import *

import time
import os
import pyttsx3
engine = pyttsx3.init()

def recog_procedure():
    #user exist so start reconizing for 30 sec
    current_user_id=0
    t0=time.perf_counter()
    while current_user_id!=-1:
        current_user_id=facerecog()
        time.sleep(1)
        print(current_user_id)
        if current_user_id!=-1:
            break
        #print(time.perf_counter()-t0)
        if int(time.perf_counter()-t0)>30:
        	break
    return current_user_id



def music_play_among_listed_songs(music_dir,songs,user_id):
    songnum=random.randint(0,len(songs)-1)
    while not('.mp3' in songs[songnum]):
        songnum=random.randint(0,len(songs)-1)
    os.startfile(os.path.join(music_dir,songs[songnum]))
    
    #add the song to the specified iser_id file
    tmp=[]
    tmp.append(songs[songnum])
    filename="users/User."+str(user_id)+"music"
    writelistString(tmp,filename)
    
    #remove the played song 
    songs.remove(songs[songnum])
    return songs

def music_control(user_id,new_old):

    music_dir = '/Users/kaustubhhalyal/Desktop/Faurecia/Music'#music path
    play=True #play song button

    if new_old=="new":
        #play the songs in the directory
        songs=[]
        songs = os.listdir(music_dir)
        while play:
            if len(songs)==0:
                #if all songs are played then repeat
                songs = os.listdir(path)
            songs=music_play_among_listed_songs(music_dir,songs,user_id)
            play=False
            tmp=input("play next song [y/n]")
            if tmp=="y":
                play=True
            elif tmp=="n":
                play==False
            
    elif new_old=="old":
    	#input the list of songs played by user previously
    	filename="users/User."+str(user_id)+"music"
    	songs=[]
    	try:
    		somgs=readlistString(filename)
    		file_not_found=False
    	except:
    		songs = os.listdir(music_dir)
    		file_not_found=True
    	while play:
    	    if len(songs)==0:
    	    	#repeat the songs
    	    	if file_not_found==False:
    	    	    somgs=readlistString(filename)
    	    	elif file_not_found==True:
    	    		songs = os.listdir(music_dir)

    	    songs=music_play_among_listed_songs(music_dir,songs,user_id)
    	    play=False
    	    tmp=input("play next song [y/n]")
    	    if tmp=="y":
    	    	play=True
    	    elif tmp=="n":
    	    	play==False 




def old_user_procedure():
    #get the stored settings and adjust as per that
    user_id=recog_procedure()
    if user_id==-1:
        user_id=new_user_procedure()
        music_control(user_id,"new")
    else:
        greet()
        filename="users/User."+str(user_id)+"initial_settings"
        #get all settings of this user id
        settings=[]
        settings=readlistString(filename)
        time.sleep(2)

        engine.say("Initializing interior adjustments")
        time.sleep(1)

        engine.say("making seat adjustments")
        time.sleep(1)

        engine.say("Adjusting steering wheel")
        time.sleep(1)

        engine.say("Adjusting side mirror")
        time.sleep(1)

        engine.say("Adjusting interior lighting")
        time.sleep(1)

        engine.say("playing music")
        music_control(user_id,"old")



def new_user_procedure():
    engine.say("Are you a new user ? and want to register")
    engine.runAndWait()
    choice=input("yes/y or no/n")
    if choice== "yes" or choice== "y":
        engine.say("Please place your face infront of camera")
        engine.runAndWait()
        #time.sleep()
        #input the first user
        #gather data set
        user_id=data_entry()
        #now train the model
        train_model()

        #set the settings for new user on his user id
        filename="users/User."+str(user_id)+"initial_settings"
        #make a structure to store line by line daata in fix format using list
        settings=[]

        #SEAT
        print("\nInput seat adjustments :")
        engine.say("Please make the seat adjustments")
        engine.say("Start adjusting")

        engine.say("adjust height")
        time.sleep(2)
        tmp=int(input("adjust height  Height : "))
        settings.append(tmp)

        engine.say("adjust transverse position")
        time.sleep(2)
        tmp=int(input("adjust transverse position   Transverse : "))
        settings.append(tmp)

        engine.say("adjust backresting position")
        time.sleep(2)
        tmp=int(input("adjust backresting position   Backrest Angle : "))
        settings.append(tmp)
        engine.say("Seat adjustment settings are stored")


        #stairning wheel
        engine.say("Please make steering wheel adjustment")
        time.sleep(2)
        tmp=int(input("Input steering wheel height adjustment   Height : "))
        settings.append(tmp)
        engine.say("steering wheel settings stored")

          
        #SIDE MIRROR
        engine.say("Please make side mirror adjustment")
        time.sleep(2)
        tmp=int(input("input side mirror angle adjustment   Angle : "))
        settings.append(tmp)
        engine.say("side mirror settings stored")


        #LIGHTING
        engine.say("Please set light intensity")
        time.sleep(2)
        tmp=int(input("set light intensity"))
        settings.append(tmp)
        engine.say("lighting settings stored")
       

        #store all setting paraeters 
        writelistString(settings,filename)
        engine.runAndWait()
        return user_id


    else:
        #contiue with manual control with initial default settings
        engine.say("Contiueing system with manual control with initial default settings")
        engine.runAndWait()
        return (-1)



def main_fun():
    #first start of system 
    #check if user exist
    id=[]
    id=readlistint("id")
    print(id)
    if len(id)>0:
        old_user_procedure()
    else:
        #input the first user
        user_id=new_user_procedure()
        if user_id !=-1:
            #start taking music test for this user
            music_control(user_id,"new")

main_fun()
