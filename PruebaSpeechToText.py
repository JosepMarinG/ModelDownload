import speech_recognition as sr


def main():
    recognizer = sr.Recognizer()

    # Usar el índice correcto del micrófono
    with sr.Microphone(device_index=4) as source:
        print("Ajustando ruido... Espera un momento.")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        print("Di algo...")

        try:
            #audio = recognizer.listen(source, timeout=25, phrase_time_limit=15)
            audio = recognizer.listen(source)
            text = recognizer.recognize_google(audio, language="es-ES")
            print("Has dicho:", text)
        except sr.WaitTimeoutError:
            print("No se detectó ninguna voz a tiempo.")
        except sr.UnknownValueError:
            print("No se pudo reconocer el audio.")
        except sr.RequestError:
            print("Error con el servicio de reconocimiento.")


if __name__ == "__main__":
    main()
