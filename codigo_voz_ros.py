#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
import subprocess
import os

class VoiceAssistantNode:
    def __init__(self):
        rospy.init_node('tictac_voice_assistant', anonymous=True)

        # Publishers (não mudam)
        self.user_speech_pub = rospy.Publisher('/user_speech', String, queue_size=10)
        self.assistant_response_pub = rospy.Publisher('/assistant_response', String, queue_size=10)

        # Configurações do reconhecedor (não mudam)
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 0.7
        self.recognizer.phrase_threshold = 0.3
        
        # --- INÍCIO DAS MUDANÇAS ---

        # 1. Adicionar uma variável de estado para controlar se o assistente está ativo
        self.is_active = False

        # 2. Criar um Subscriber para o tópico /detected_color
        #    Ele vai chamar a função 'self.color_callback' sempre que uma nova cor for publicada
        rospy.Subscriber('/detected_color', String, self.color_callback)

        rospy.loginfo("Nó de assistente de voz iniciado. Aguardando ativação pela cor verde.")

        # --- FIM DAS MUDANÇAS ---

    def color_callback(self, msg):
        """
        Esta função é chamada toda vez que o nó detector_cores publica uma mensagem.
        """
        detected_color = msg.data
        
        # Verifica se a cor detectada contém "Vermelho" (funciona para "Vermelho1" e "Vermelho2")
        is_green_detected = "Verde" in detected_color

        # Lógica para ATIVAR o assistente
        # Se a cor for vermelha E o assistente estiver inativo, ative-o.
        if is_green_detected and not self.is_active:
            rospy.loginfo("Cor verde detectada! Ativando o assistente de voz.")
            self.is_active = True
            # Toca um som de ativação ou fala uma saudação
            self.falar("Oi! Me chamo TicTac. Como posso ajudar?")

        # Lógica para DESATIVAR o assistente (opcional, mas recomendado)
        # Se a cor não for vermelha E o assistente estiver ativo, desative-o.
        elif not is_green_detected and self.is_active:
            rospy.loginfo("Cor verde não está mais visível. Desativando o assistente.")
            self.is_active = False
            # self.falar("Desativando. Mostre a cor vermelha para falar comigo novamente.")

    # A função 'falar' não precisa de alterações
    def falar(self, texto):
        rospy.loginfo("TicTac respondendo: " + texto)
        try:
            self.assistant_response_pub.publish(texto)

            audio_fala = "/tmp/fala.mp3"
            tts = gTTS(text=texto, lang='pt-br')
            tts.save(audio_fala)

            subprocess.run(
                ["mpg123", "-q", audio_fala],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            os.remove(audio_fala)

        except Exception as e:
            rospy.logerr(f"Erro ao gerar ou reproduzir a fala: {e}")

    # A função 'tocar_musica' não precisa de alterações
    def tocar_musica(self, nome_arquivo):
        if os.path.exists(nome_arquivo):
            try:
                rospy.loginfo(f"Tocando música: {nome_arquivo}")

                subprocess.run(
                    ["mpg123", "-q", nome_arquivo],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

            except Exception as e:
                rospy.logerr(f"Erro ao tocar a música '{nome_arquivo}': {e}")
                self.falar("Desculpe, tive um problema para tocar a música.")
        else:
            rospy.logwarn(f"Erro: arquivo '{nome_arquivo}' não encontrado.")
            self.falar(f"Eu adoraria, mas não encontrei o arquivo de música necessário.")

    
    def listen_and_process(self):
        # Esta função agora só será chamada se self.is_active for True
        with sr.Microphone() as source:
            rospy.loginfo("Ouvindo...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10) # Adicionado timeouts

            try:
                text = self.recognizer.recognize_google(audio, language='pt-BR')
                text_lower = text.lower()
                rospy.loginfo("Transcrição: " + text_lower)
                self.user_speech_pub.publish(text_lower)

                # --- Lógica da conversação (sem alterações) ---
                if "tchau" in text_lower or "encerrar" in text_lower or "sair" in text_lower:
                    self.falar("Até mais! Foi um prazer conversar com você")
                    self.is_active = False # Em vez de encerrar o nó, apenas o desativa
                
                elif "bom dia" in text_lower:
                    self.falar("Bom dia, como vai?")
                
                elif "mimosa" in text_lower:
                    self.falar("Na verdade não me chamo Tictac, sou o Pablo Escobar disfarçado.")
                    self.tocar_musica("pablo.mp3")
                    self.is_active = False 
                
                elif "boa tarde" in text_lower:
                    self.falar("Olá, boa tarde, como vai?")
                
                elif "boa noite" in text_lower:    
                    self.falar("Tenha uma boa noite")

                elif "dançar" in text_lower or "dança" in text_lower:
                    self.falar("Claro! Vamos dançar!")
                    self.tocar_musica("balaomagico.mp3")

                elif "alarme" in text_lower or "horário" in text_lower:    
                    self.falar("Tenha uma boa noite")
                
                else:
                    self.falar("Que legal saber disso!")
                    self.is_active = False # Em vez de encerrar o nó, apenas o desativa

            except sr.UnknownValueError:
                rospy.logwarn("Não foi possível entender o áudio. Tente novamente.")
            except sr.RequestError as e:
                rospy.logerr(f"Erro no serviço de reconhecimento do Google; {e}")
            except sr.WaitTimeoutError:
                rospy.loginfo("Nenhuma fala detectada. Continuo aguardando.")

    def run(self):
        # Removido o 'falar' inicial daqui
        
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # --- MUDANÇA CRÍTICA ---
            # Só executa a lógica de ouvir se o estado for ativo
            if self.is_active:
                self.listen_and_process()
            
            # Mesmo se estiver inativo, o loop continua para que o callback possa funcionar
            rate.sleep()

if __name__ == '__main__':
    try:
        assistant = VoiceAssistantNode()
        assistant.run()
    except rospy.ROSInterruptException:
        pass
