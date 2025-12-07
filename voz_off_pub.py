#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from unidecode import unidecode
from vosk import Model, KaldiRecognizer
import pyaudio
import json
import subprocess
import os

class TicTacVoiceMaster:
    def __init__(self):
        rospy.init_node("tictac_voz_master")

        # --- 1. CONFIGURACOES ---
        self.model_path = rospy.get_param('~model_path', '/home/sirigueijo/model')
        self.audio_path = rospy.get_param('~audio_path', '/home/sirigueijo/siricascudo_ws/src/aborgue/audios')
        
        # Variavel de controle: Comeca 'dormindo'
        self.ativo = False

        # --- 2. MAPA MESTRE (Ouvido -> Som + Comando ROS) ---
        # ATENCAO: Todas as chaves em minusculo e SEM ACENTO
        self.dicionario = {
            # Frases de interacao
            "dia":          ["bomdia.mp3",   "BOM_DIA"],
            "tarde":        ["boatarde.mp3", "BOA_TARDE"],
            "noite":        ["boanoite.mp3", "BOA_NOITE"],
            "mimosa":       ["escobar.mp3",  "MIMOSA"],
            "grupo b":      ["sirigueijo.mp3", "GRUPO_B"],
            
            # Comandos de acao 
            "dancar":       ["dancar.mp3",   "DANCAR"],
            "andar":        ["andar.mp3",    "ANDAR"],
            
            # Comandos de saida
            "tchau":        ["tchau.mp3",    "DESATIVAR"],
            "encerrar":     ["tchau.mp3",    "DESATIVAR"],
            "sair":         ["tchau.mp3",    "DESATIVAR"],
            
            # Comandos extras do seu primeiro codigo
            "mimosa":        ["escobar.mp3",  "PABLO"],
            "cade":          ["capixaba.mp3",  "CAPIXABA"],
        }

        self.audio_ativacao = "oitictac.mp3"

        # --- 3. COMUNICACAO ROS ---
        self.pub = rospy.Publisher('/comandos', String, queue_size=10)
        self.sub = rospy.Subscriber('/color_detected', String, self.callback_cor)

        # --- 4. CONFIGURACAO VOSK ---
        rospy.loginfo("Carregando modelo Vosk...")
        
        try:
            self.model = Model(self.model_path)
            
            # Cria a gramatica apenas com as chaves sem acento
            palavras_chave = list(self.dicionario.keys())
            grammar = json.dumps(palavras_chave)
            
            self.rec = KaldiRecognizer(self.model, 16000, grammar)
        except Exception as e:
            rospy.logerr(f"Erro ao carregar modelo: {e}")
            return

        # --- 5. MICROFONE ---
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )
        self.stream.start_stream()

        rospy.loginfo("Sistema PRONTO! Mostre a cor VERDE para iniciar.")

    def tocar_audio(self, nome_arquivo):
        """ Toca o audio usando mpg123 em background """
        if not nome_arquivo: return

        caminho = os.path.join(self.audio_path, nome_arquivo)
        if os.path.exists(caminho):
            # O subprocess.Popen nao trava o codigo
            subprocess.Popen(["mpg123", "-q", caminho])
        else:
            rospy.logwarn(f"Audio nao encontrado: {caminho}")

    def callback_cor(self, msg):
        """ Ativa o sistema se vir verde """
        # unidecode garante que o texto de entrada fique sem acento
        cor_detectada = unidecode(msg.data.lower())
        
        if "verde" in cor_detectada and not self.ativo:
            self.ativo = True
            rospy.loginfo(">>> ATIVADO POR COR VERDE <<<")
            self.tocar_audio(self.audio_ativacao)
            
            # Pausa para nao ouvir o proprio som de 'oi'
            rospy.sleep(2.0) 
            
            # Limpa o buffer de audio
            if self.stream.is_active():
                self.stream.stop_stream()
                self.stream.start_stream()

    def loop(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if not self.ativo:
                rate.sleep()
                continue

            try:
                data = self.stream.read(1024, exception_on_overflow=False)

                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.Result())
                    texto_bruto = result.get("text", "")
                    
                    if texto_bruto:
                        # O segredo: unidecode tira os acentos da voz ouvida
                        # Ex: "dançar" vira "dancar" e bate com o dicionario
                        texto = unidecode(texto_bruto.lower())
                        rospy.loginfo(f"Ouvi: {texto}")

                        if texto in self.dicionario:
                            dados = self.dicionario[texto]
                            arquivo_som = dados[0]
                            comando_ros = dados[1]

                            # 1. Publica
                            self.pub.publish(comando_ros)
                            rospy.loginfo(f"Publicado: {comando_ros}")

                            # 2. Toca som
                            self.tocar_audio(arquivo_som)

                            # 3. Verifica se e para desligar
                            if comando_ros == "DESATIVAR":
                                rospy.loginfo("Desativando sistema...")
                                self.ativo = False
                                rospy.sleep(1.0)
                        
            except IOError:
                pass
            
            rate.sleep()

    def cleanup(self):
        rospy.loginfo("Limpando recursos...")
        if hasattr(self, 'stream'): 
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'): 
            self.audio.terminate()

if __name__ == "__main__":
    node = TicTacVoiceMaster()
    try:
        node.loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
