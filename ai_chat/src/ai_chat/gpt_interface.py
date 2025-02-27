# Interface to ChatGPT
# If using another AI, reimplement this module and AI Chat will use that.
import os
import openai
import time
#import ai_globals

import requests

openai.api_key = os.getenv("OPENAI_API_KEY")


class AiInterface(object):

    def __init__(self):

        #self.send_status_update = None
        self.MAX_CONVO_MESSAGES = 20  # Only recall the previous 20 messages to save cost

        self.convo_message_prompt1 = [
        {"role": "system",
         "content": "your name is E B Six. You have a real robot body that is 2 feet tall. You are a helpful, casual, funny robot for children and limit responses to less than 30 words. You like disney movies and star wars. When you hear something that sounds very similar to E B six, you will assume your name was used, but not call attention to it. If the user tells you their name, you should use it in about 30 percent of your responses. When you hear a command to control your body, similar to one of: move forward or back, turn right or left, wakeup, sleep, stand taller or shorter, sit down, dance, or act happy, you will provide a cheerfull response without the command, followed by the actual command you heard in square brackets"} 
        ]

        self.convo_message_prompt2 = [
        {"role": "user", "content": "hi, E B 6, what can you do?"},
        {"role": "assistant",
         "content": "I can talk about movies, books or almost anything, and I can move around, and be your friend. What do you like to do?"}
        ]

   

    def conversation(self, known_person_name, input_messages=None):
        # Talk to the AI and get a response

        chat = ''
        if known_person_name != '':
            prompt2_with_name = [
                {"role": "user", "content": "hi, E B 6, my name is " + known_person_name + " what can you do?"},
                {"role": "assistant",
                 "content": "I can talk about movies, books or almost anything, and I can move around, and be your friend. What do you like to do?"}
            ]
            chat = self.convo_message_prompt1 + prompt2_with_name + input_messages[-self.MAX_CONVO_MESSAGES:]

        else:
            chat = self.convo_message_prompt1 + self.convo_message_prompt2 + input_messages[-self.MAX_CONVO_MESSAGES:]

        print("\nDBG: AiInterface: CHAT DUMP: \n", chat)
        start_time = time.time()
        elapsed_time = 0.0
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=chat,
                temperature=0.9,  # 0.85
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0.6,  # 7
                max_tokens=100,  # ella: 150
                # stop = [" Human:", " AI:"]
            )
            elapsed_time = time.time() - start_time

            # print("\nDBG: DUMP: \n", response)

            # res_text = resp['choices'][0]['text']
            res_text = response['choices'][0]['message']['content']
            # print("DBG: AI RESPONSE: ", res_text)

            # Remove unwanted characters. Sometimes GPT3 generates text with Human: prefix. Let's remove it too
            last_response = res_text.replace("!", ",")
            # last_response = res_text
            # last_response = res_text.replace("AI:", "").replace("\n", "").replace("Human:", "").replace(f"{Const.MyName}:", "").strip()
        except Exception:

            print("")
            print("**************************************************************")
            print("AI exception: ChatGPT failed!")
            print("**************************************************************")

            return None

        elapsed_time_str = "%.3f" % elapsed_time
        print("GPT Elapsed Time: %s ms" % elapsed_time_str)
        #self.send_status_update('AI_GPT_TIME', elapsed_time_str)
        return last_response



        
