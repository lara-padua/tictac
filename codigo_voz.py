import speech_recognition as sr
from gtts import gTTS #biblioteca que dá a voz do google
import playsound3
import os #biblioteca para interagir com o sistema operacional // remover o arquivo de áudio

resposta = ""
r = sr.Recognizer() # criar um objeto de reconhecimento
r.pause_threshold = 0.7 #define uma pausa de 1 segundo para parar a execução
r.phrase_threshold = 0.3 #qualquer fala que dure menos que 0,3 segundos é descartada

def falar(texto):
    try:
        tts = gTTS(text=texto, lang='pt-br')
        tts.save('arquivo_audio')
        playsound3.playsound("arquivo_audio") # reproduzir o áudio gerado
        os.remove("arquivo_audio") #deleta o áudio para que não acumule memória 
    except Exception as e:
        print(f"Erro ao gerar ou reproduzir o áudio: {e}")

falar("Oi, me chamo TicTac! Como posso te ajudar?")
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
                resposta = "Bom dia, como vai?"
                falar(resposta)
            elif "mimosa" in text_lower:
                resposta = "Na verdade não me chamo Tictac, sou o Pablo Escobar disfarçado"
                falar(resposta)
                arquivo_musica = "pablo.mp3"
                if os.path.exists(arquivo_musica):
                    try:
                        print(f"Tocando: {arquivo_musica}")
                        # Toca o arquivo de música
                        playsound3.playsound(arquivo_musica)
                        break
                    except Exception as e:
                        print(f"Erro ao tocar a música: {e}")
                        falar("Desculpe, tive um problema para tocar a música.")
                else:
                    # Se não encontrar o arquivo, avisa o usuário
                    print(f"Erro: arquivo '{arquivo_musica}' não encontrado.")
                    falar(f"Eu adoraria, mas não encontrei o arquivo {arquivo_musica} na pasta.")

            elif "boa tarde" in text_lower:
                resposta = "Olá, boa tarde, como vai?"
                falar(resposta)
            elif "boa noite" in text_lower:    
                resposta ="Tenha uma boa noite"
                falar(resposta)
            elif "dançar" in text_lower:
                falar("Claro!Vamos dançar!")
                arquivo_musica = "balaomagico.mp3"
                if os.path.exists(arquivo_musica):
                    try:
                        print(f"Tocando: {arquivo_musica}")
                        # Toca o arquivo de música
                        playsound3.playsound(arquivo_musica)
                    except Exception as e:
                        print(f"Erro ao tocar a música: {e}")
                        falar("Desculpe, tive um problema para tocar a música.")
                else:
                    # Se não encontrar o arquivo, avisa o usuário
                    print(f"Erro: arquivo '{arquivo_musica}' não encontrado.")
                    falar(f"Eu adoraria, mas não encontrei o arquivo {arquivo_musica} na pasta.")
            else:
                resposta = "Que bom saber disso!"
                falar(resposta)
                break


        except sr.UnknownValueError:
            print("Não foi possível entender o áudio")
            
        except sr.RequestError as e:
            print("Erro ao solicitar resultados do serviço de reconhecimento de voz do Google; {0}".format(e))