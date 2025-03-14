#!/usr/bin/env python3
# Sysmon GUI Layout for PySimpleGui / FreeSimpleGui
#import PySimpleGUI as psg
import FreeSimpleGUI as psg


layout = [

    [
        psg.Text('No Battery', enable_events=True, key='-BATTERY-', size=(11, 1),
            expand_x=False, justification='center', font=('Arial Bold', 12), text_color='white',  ), 

        psg.Button("0", enable_events=True, key='-POSE0-'),
        psg.Button("1", enable_events=True, key='-POSE1-'),
        psg.Button("2", enable_events=True, key='-POSE2-'),
        psg.Button("3", enable_events=True, key='-POSE3-'),
        psg.Button("4", enable_events=True, key='-POSE4-'),
        psg.Button("5", enable_events=True, key='-POSE5-'),
    ],

    [
        psg.Text('', enable_events=False, size=(10, 1),
            expand_x=False, justification='center', font=('Arial Bold', 10), ), 
    ],
    
    [   # Lights
        psg.Button("Eyes On",   enable_events=True, size=(4, 2), key='-EYES_ON-', font=('Arial Bold', 10), expand_x=True,),
        psg.Button("Ears On",   enable_events=True, size=(4, 2), key='-EARS_ON-', font=('Arial Bold', 10), expand_x=True,),
        psg.Button("Body On",   enable_events=True, size=(4, 2), key='-BODY_ON-', font=('Arial Bold', 10), expand_x=True,),

        psg.Button("Eyes Off",  enable_events=True, size=(4, 2), key='-EYES_OFF-', font=('Arial Bold', 10), expand_x=True,),
        psg.Button("Ears Off",  enable_events=True, size=(4, 2), key='-EARS_OFF-', font=('Arial Bold', 10), expand_x=True,),
        psg.Button("Body Off",  enable_events=True, size=(4, 2), key='-BODY_OFF-', font=('Arial Bold', 10), expand_x=True,),
    ],

    [   # Behaviors
        psg.Button("Head Zero", enable_events=True, size=(7, 2), key='-HEAD_CENTER-', ),
        psg.Button("Wake",      enable_events=True, size=(7, 2), key='-WAKE-'),
        psg.Button("Sleep",     enable_events=True, size=(7, 2), key='-SLEEP-'),
        psg.Button("Stop",      enable_events=True, size=(7, 2), key='-STOP-'),
    ],
    
    [   # Behaviors
        psg.Button("Follow",    enable_events=True, size=(7, 2), key='-FOLLOW-', ),
        psg.Button("Danger",    enable_events=True, size=(7, 2), key='-DANGER-'),
        psg.Button("Dance",     enable_events=True, size=(7, 2), key='-DANCE-'),
        psg.Button("Happy",     enable_events=True, size=(7, 2), key='-HAPPY-'),
    ],
    
    [
        psg.Text('', enable_events=False, size=(10, 1),
            expand_x=False, justification='center', font=('Arial Bold', 10), ), 
    ],


    ########################## STATUS ##########################
    # Status Header
    [     
        psg.Text('ITEM', key='-Name1-', size=(20, 1), enable_events=True, expand_x=False, 
            justification='center',  #relief="sunken", 
        ),
        psg.Text('STATUS', key='-Name2-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', #relief="sunken",
        ),
    ],

  
    [     
        psg.Text('AI STATE:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-ai_state-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('VOICE:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-voice_state-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('SPEECH RECO:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-speech_reco_state-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('AI GPT TIME:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-ai_gpt_time-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('AI NAME:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-ai_name-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],

    [     
        psg.Text('BEHAVIOR:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-behavior_mode-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],

    [     
        psg.Text('FACE TRACKER:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-face_tracker_status-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('FACE DETECTOR:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-face_detector_status-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('DEPTH CAMERA:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-depth_camera_status-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('OBJECT FRONT:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-object_front-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('OBJECT REAR:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-object_rear-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],

    
    [     
        psg.Text('BODY LIGHTS MODE:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-body_light_mode-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
    [     
        psg.Text('BLUETOOTH PHONE:', size=(20, 1), expand_x=False, 
            justification='left', relief="sunken", text_color='black', background_color = "LightGrey", 
        ),
        psg.Text(' 0 ', key='-bluetooth_phone_status-', size=(25, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],




    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 12),
        ),
    ],

        
    [   # Speech Recognition header 
        psg.Text('Speech Recognition:', size=(60, 1), expand_x=False, 
            justification='left', text_color='white', # background_color = "LightGrey", 
        ),
    ],
    
    [   # Speech recognition text 
        psg.Text(' 0 ', key='-speech_reco_text-', size=(60, 1), enable_events=True, 
            expand_x=False, justification='center', relief="sunken", text_color='black', background_color = "LightGrey",
        ),
    ],
    
  
    
]


