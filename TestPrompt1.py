from transformers import AutoModelForCausalLM, AutoTokenizer

model_name = "EleutherAI/gpt-j-6B"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForCausalLM.from_pretrained(model_name)

# Define un prompt inicial
prompt = "Hola, ¿cómo estás?"
inputs = tokenizer(prompt, return_tensors="pt")

# Genera texto a partir del prompt
outputs = model.generate(**inputs, max_new_tokens=50)
generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)

print(generated_text)
