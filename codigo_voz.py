import speech_recognition as sr
from gtts import gTTS #biblioteca que dá a voz do google
import playsound3
import os #biblioteca para interagir com o sistema operacional // remover o arquivo de áudio

resposta = ""
r = sr.Recognizer() # criar um objeto de reconhecimento
r.pause_threshold = 1.0 #define uma pausa de 1 segundo para parar a execução
r.phrase_threshold = 0.3 #qualquer fala que dure menos que 0,3 segundos é descartada

def falar(texto):
    try:
        tts = gTTS(text=texto, lang='pt-br')
        tts.save('arquivo_audio')
        playsound3.playsound("arquivo_audio") # reproduzir o áudio gerado
        os.remove("arquivo_audio") #deleta o áudio para que não acumule memória 
    except Exception as e:
        print(f"Erro ao gerar ou reproduzir o áudio: {e}")

falar("Trala la la. Oi, me chamo TicTac! Como posso te ajudar?")

while True:
    with sr.Microphone() as source: # capturar áudio do microfone
        print("Ouvindo...")
        r.adjust_for_ambient_noise(source, duration = 0.5) #diminiu o barulho do ambiente
        audio = r.listen(source)

        text = r.recognize_google(audio, language='pt-BR')
        text_lower = text.lower() #transforma todas as letras em minúsculas
        print("Transcrição: " + text_lower)

        try:
            if "tchau" in text_lower or "encerrar" in text_lower or "sair" in text_lower:
                resposta = "Até mais! Foi um prazer conversar com você."
                falar(resposta)
                break  # Sai do loop while True

            elif "bom dia" in text_lower:
                resposta = "Bom dia, tudo bem?"
                falar(resposta)
            elif "boa tarde" in text_lower:
                resposta = "Olá, boa tarde, como vai?"
                falar(resposta)
            elif "boa noite" in text_lower:    
                resposta ="Tenha uma boa noite"
                falar(resposta)
            elif "batimentos" in text_lower:
                resposta = "Vamos checar seus batimentos"
                falar(resposta)
            else:
                resposta = "Você disse:" + text
                falar(resposta)


        except sr.UnknownValueError:
            print("Não foi possível entender o áudio")
            
        except sr.RequestError as e:
            print("Erro ao solicitar resultados do serviço de reconhecimento de voz do Google; {0}".format(e))