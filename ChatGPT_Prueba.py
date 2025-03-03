from openai import OpenAI

client = OpenAI(
  api_key="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"
)

completion = client.chat.completions.create(
  model="gpt-3.5-turbo",
  store=True,
  messages=[
    {"role": "user", "content": "Que es GPT?"}
  ]
)

print(completion.choices[0].message.content);


"""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"""
"""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"""

"""Tengo un robot que se mueve al enviarle el comando ={x=1.0,y=1.0, z=0.0} este comando le especifica la velocidad de movimiento del robot en m/s.
Tambien reacciona al comando wait(1.0) que se encarga de esperar un tiempo en segundos.
Quiero que tu respuesta sea solo comandos del robot.
Hazme un cuadrado."""

from openai import OpenAI

client = OpenAI(
  api_key=""
)

completion = client.chat.completions.create(
  model="gpt-3.5-turbo",
  store=True,
  messages=[
    {"role": "user", "content": "Que es GPT?"}
  ]
)

print(completion.choices[0].message.content);