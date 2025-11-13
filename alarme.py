import speech_recognition as sr
from gtts import gTTS
import time
import datetime
import os
import threading
from playsound3 import playsound
import re 



def falar(texto):
    """Converte texto em fala e reproduz o áudio."""
    print(f"🤖 {texto}")
    try:
        tts = gTTS(texto, lang='pt-br')
        # Salva o arquivo com um nome único para evitar conflitos
        nome_arquivo = "fala_temp.mp3"
        tts.save(nome_arquivo)
        playsound(nome_arquivo)
        os.remove(nome_arquivo)
    except Exception as e:
        print(f"Erro na função 'falar': {e}")


# --- FUNÇÃO CORRIGIDA ---
def texto_para_numero(frase):
    """
    Converte uma frase falada em (hora, minuto).
    NOVA VERSÃO: Agora lida tanto com dígitos (ex: "8:30") quanto com palavras (ex: "oito e trinta").
    """
    frase = frase.lower().strip()

    # --- ETAPA 1: Tentar extrair números (dígitos) primeiro ---
    # Procura por padrões como "8:30", "08:30" ou até "8 30"
    match = re.search(r'(\d{1,2})[:\s](\d{1,2})', frase)
    if match:
        hora = int(match.group(1))
        minuto = int(match.group(2))
        return hora, minuto

    # Se o padrão acima falhar, procura por números soltos na frase
    # Ex: "alarme para 8 e 30" -> encontrará ['8', '30']
    numeros_encontrados = re.findall(r'\d+', frase)
    if len(numeros_encontrados) >= 2:
        hora = int(numeros_encontrados[0])
        minuto = int(numeros_encontrados[1])
        return hora, minuto
    elif len(numeros_encontrados) == 1:
        hora = int(numeros_encontrados[0])
        minuto = 0
        return hora, minuto

    # --- ETAPA 2: Se não encontrou dígitos, usar a lógica original de palavras ---
    palavras = frase.split()
    numeros = {
        "zero": 0, "um": 1, "uma": 1, "dois": 2, "duas": 2, "três": 3, "tres": 3,
        "quatro": 4, "cinco": 5, "seis": 6, "sete": 7, "oito": 8, "nove": 9,
        "dez": 10, "onze": 11, "doze": 12, "treze": 13, "catorze": 14, "quatorze": 14,
        "quinze": 15, "dezesseis": 16, "dezessete": 17, "dezoito": 18, "dezenove": 19,
        "vinte": 20, "trinta": 30, "quarenta": 40, "cinquenta": 50
    }
    
    # Lógica original para palavras (com pequenas melhorias)
    valores_numericos = [numeros[p] for p in palavras if p in numeros]
    hora = 0
    minuto = 0

    if len(valores_numericos) >= 2:
        # Lógica para tratar "vinte e um" -> 21
        if valores_numericos[0] in [20, 30, 40, 50] and valores_numericos[1] < 10:
             hora = valores_numericos[0] + valores_numericos[1]
             if len(valores_numericos) > 2:
                 minuto = valores_numericos[2]
             else:
                 minuto = 0
        else:
            hora = valores_numericos[0]
            minuto = valores_numericos[1]
    elif len(valores_numericos) == 1:
        hora = valores_numericos[0]
        minuto = 0

    return hora, minuto


def alarme(hora, minuto, som="/home/beatriz/Documents/tictac_ws/src/segundo_pacote/alarme-samsung.mp3"):
    """Função que monitora o tempo e dispara o alarme na hora certa."""
    falar(f"Alarme ativado para {hora:02d}:{minuto:02d}. Pode relaxar!")
    while True:
        agora = datetime.datetime.now()
        if agora.hour == hora and agora.minute == minuto:
            falar("Está na hora! Alarme tocando!")
            if os.path.exists(som):
                # O playsound pode bloquear, então é melhor rodá-lo em uma thread separada
                # para que a assistente possa continuar falando, se necessário.
                threading.Thread(target=playsound, args=(som,)).start()
            else:
                falar(f"Aviso: Arquivo de som do alarme '{som}' não foi encontrado.")
            break
        time.sleep(5) # Verificando a cada 5 segundos é suficiente e mais leve.

def definir_alarme():
    """Função principal que gerencia a conversa para definir o alarme."""
    recognizer = sr.Recognizer()
    while True:
        with sr.Microphone() as source:
            print("\nOuvindo o horário...")
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                # É uma boa prática colocar o listen() dentro do try/except também
                audio = recognizer.listen(source, timeout=10, phrase_time_limit=10)
            except sr.WaitTimeoutError:
                falar("Eu não ouvi nada. Se quiser definir um alarme, é só falar.")
                continue

        try:
            resposta = recognizer.recognize_google(audio, language='pt-BR')
            print(f"Você disse: {resposta}")

            hora, minuto = texto_para_numero(resposta)
            
            # Validação para evitar horários impossíveis
            if not (0 <= hora <= 23 and 0 <= minuto <= 59):
                falar(f"O horário {hora} e {minuto} não parece válido. Vamos tentar de novo.")
                continue

            falar(f"Você quer definir o alarme para {hora:02d}:{minuto:02d}?")
            with sr.Microphone() as confirm_source:
                print("Ouvindo confirmação...")
                recognizer.adjust_for_ambient_noise(confirm_source, duration=0.5)
                confirm_audio = recognizer.listen(confirm_source, timeout=5, phrase_time_limit=3)
                confirm_text = recognizer.recognize_google(confirm_audio, language='pt-BR').lower()

            if "sim" in confirm_text or "isso" in confirm_text or "confirmo" in confirm_text or "quero" in confirm_text:
                threading.Thread(target=alarme, args=(hora, minuto)).start()
                break
            elif "não" in confirm_text or "nao" in confirm_text:
                falar("Tudo bem, cancelado. Por favor, diga o novo horário.")
                continue
            else:
                falar("Não entendi sua resposta. Por favor, diga o horário novamente.")
        
        # --- EXCEÇÕES ESPECÍFICAS ---
        except sr.UnknownValueError:
            falar("Desculpe, não consegui entender o que você disse. Pode repetir o horário?")
        except sr.RequestError as e:
            falar("Não consegui me conectar ao serviço de reconhecimento. Verifique sua internet.")
            print(f"Erro de API: {e}")
            break # Sai do loop se não há conexão
        except Exception as e:
            print(f"Ocorreu um erro inesperado: {e}")
            falar("Ocorreu um erro. Vamos tentar de novo.")


# 🚀 Código principal
falar("Olá! Vamos definir um alarme por voz.\n Diga o horário!.")
definir_alarme()

