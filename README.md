# Observações

- Você pode ver uma demonstração do projeto [aqui!](https://alinsperedu-my.sharepoint.com/:v:/g/personal/caioesr_al_insper_edu_br/ESUaMyX2Ow9LsnEpp8sGcL0BTGX1HTEL6Jg31N0E9jBs_g?e=KWYMVL)
- Se quiser as credenciais do Google Cloud Speech API pode pedir ao Caio/Diego ou ver [esse tutorial](https://medium.com/codex/google-speech-to-text-api-tutorial-with-python-2e049ae3f525).

# Como funciona

Resumidamente, os passos do projeto são os seguintes:

1. Apertar o botão/simplesmente falar e acionar o Gate
2. Falar o comando (ligar/desligar/piscar)
3. Os valores do AFEC são gravados na SDRam
4. Ler a SDRam e printar isso no buffer
5. Ler o buffer e salvar os dados numa lista no Python
6. Converter a lista de dados para um arquivo .wav
7. Enviar o arquivo para a API do Google
8. Receber a transcrição
9. Stemmizar (pegar os radicais) das palavras na transcrição
10. Buscar os radicais que equivalem a cada possível comando
11. Enviar o comando pelo Serial

# Como melhorar

## Gate

Um dos problemas principais do projeto é quando gravar a voz e quando transcrevê-la. Se for configurado no código principal que USE_GATE então a voz será gravada toda vez que o microfone detectar algum som e a pergunta é se devemos transcrevê-lo ou não. Usar o botão resolve isso mas tira um pouco do "acionado por voz" para "semicontrolado por voz". Algumas maneiras de melhorar isso utilizando o Gate é:

- Utilizar algum método p/ identificar que "feijoada" está contida na frase antes de transcrever (transformada de fourier, por exemplo)
    - O problema desse método é que a transformada de fourier é muito específica e não conseguimos generalizar para pronuncias diferentes (mais lentas, mais rápidas, mais puxadas, mais graves, mais agudas, etc)
- Transcrever sempre (como está agora)
    - O problema é o gasto, desnecessário, com requisições

## Printf

Outra possibilidade é substituir o "printf" por "uart_send" para agilizar a transmissão dos dados entre a SDRam e o Python