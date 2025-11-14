#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import subprocess
import os

def falar(texto):
    """Gera fala e reproduz sem ruído."""
    tts = gTTS(text=texto, lang='pt-br')
    arquivo = "/tmp/fala.mp3"
    tts.save(arquivo)

    # Reproduz o áudio sem mensagens no terminal
    subprocess.run(["mpg123", "-q", arquivo], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    os.remove(arquivo)

def callback(msg):
    cor = msg.data
    rospy.loginfo(f"Cor recebida: {cor}")
    if cor == "Verde":
        falar("Oi, eu sou a Tic-Tac")

def main():
    rospy.init_node("voz_tictac", anonymous=True)
    rospy.Subscriber("/detected_color", String, callback)
    rospy.loginfo("Assistente de voz aguardando a cor verde...")
    rospy.spin()

if __name__ == "__main__":
    main()
