# Software Embarcado ESP32 — Thomas, o Trem

Firmware de controle embarcado para o carrinho autônomo (Projeto Integrador 2). Implementado em C (ESP-IDF / FreeRTOS).

## 1. Como Preparar e Compilar

Se estiver usando o **VS Code**, utilize a extensão **ESP-IDF**. Se estiver usando terminal:

```bash
# Entre na pasta do projeto
cd software-embarcado-esp32

# Configure suas credenciais de Wi-Fi (Veja a seção 1.1 abaixo)
idf.py menuconfig

# Compile e grave na placa (substitua a COM_PORT)
idf.py build
idf.py -p COM_PORT flash monitor
```

### 1.1. Configurando as Credenciais de Wi-Fi

Para que o robô consiga entrar na sua rede Wi-Fi local e se comunicar com o Dashboard Web, você deve configurar o Nome da Rede (SSID) e a Senha.

1. No terminal, execute o comando:
   ```bash
   idf.py menuconfig
   ```
2. Um menu azul e cinza aparecerá. Navegue usando as setas do teclado para baixo até encontrar a opção **`Component config`** e aperte `Enter`.
3. Role a tela para baixo até encontrar **`WiFi Station`** e aperte `Enter`.
4. Selecione **`WiFi SSID`**, aperte `Enter`, digite o nome exato da sua rede (respeitando maiúsculas e minúsculas) e aperte `Enter` para confirmar.
5. Selecione **`WiFi Password`**, aperte `Enter`, digite a senha da sua rede e aperte `Enter`. **Cuidado com espaços sobrando no final!**
6. Aperte a tecla **`S`** para salvar as configurações (`Save`) e confirme com `Enter`.
7. Aperte a tecla **`Q`** para sair (`Quit`).

> **Importante:** O ESP32 só consegue se conectar a redes **2.4 GHz**. Se sua rede for de 5 GHz ou mista e houver falhas repetidas na conexão (`Desconectado; tentando reconectar...`), roteie a internet temporariamente usando a banda de 2.4 GHz do seu celular para testes.

---

## 2. Passo a Passo: Teste Unitário (Lógica Matemática)

Implementamos testes unitários baseados no **Unity Framework** para garantir que as lógicas matemáticas críticas (Cálculo do PID e Cálculo do Centro da Linha) funcionem livre de bugs físicos.

1. Abra o arquivo `main/main.c`.
2. Altere a linha 17 de `#define RUN_UNIT_TESTS 0` para `#define RUN_UNIT_TESTS 1`.
3. Compile e rode: `idf.py flash monitor`.
4. O terminal mostrará o relatório do Unity listando os testes que passaram (ex: `test_pid_anti_windup`, `test_line_follower_center`).
5. Ao finalizar, **volte para 0** para rodar o código padrão.

---

## 3. Passo a Passo: Teste Isolado de Hardware (Na Bancada)

Se um motor girar ao contrário ou houver curtos, você não quer descobrir isso com o carro correndo no chão. As rotinas isoladas servem para testar os pinos ignorando o Wi-Fi e os sensores.

1. Coloque o robô sobre um cavalete (com as rodas no ar).
2. Abra `main/main.c` e altere a linha 18 para `#define TEST_MODE_MOTOR 1`.
3. Compile e rode: `idf.py flash monitor`.
4. Observe o terminal. O robô entrará num loop girando o motor esquerdo para frente, parando, para trás, depois o motor direito, e, por fim, subirá e descerá o atuador linear.
5. Verifique visualmente se:
   - "Frente" roda a roda pra frente. Se rodar para trás, inverta os fios PWM ou solde os cabos do motor invertidos no borne da Ponte H.
   - Pinos validados pela eletrônica (LEDC): **L_FWD=18, L_REV=19, R_FWD=17, R_REV=16, EN=32**
   - Os freios param bruscamente a roda.
6. Terminado o teste e tudo ok, **volte para 0**.

---

## 4. Passo a Passo: Como Rodar o Modo Autônomo e a Interface Web

Para testar no chão de verdade e fazê-lo seguir a linha, utilize nosso Dashboard de Controle de Robótica:

1. Assegure-se que `RUN_UNIT_TESTS` e `TEST_MODE_MOTOR` no `main.c` estão em `0`.
2. Ligue o robô. Acompanhe no monitor serial da ESP-IDF até aparecer **"Aguardando IP do Roteador..."** e depois **"HTTP API em http://192.168.x.x"**.
3. Em um terminal no seu computador (na mesma rede Wi-Fi), inicie a aplicação Web:
   ```bash
   cd ../software-interface-web
   npm install   # caso seja a primeira vez
   npm run dev
   ```
4. Abra o link gerado no navegador (geralmente `http://localhost:5173`).
5. Digite o IP da ESP32 que você pegou no passo 2 dentro do campo de **Conexão** e clique em conectar. O Dashboard vai ficar Online.
6. **Calibração (Obrigatório):** Pelo painel Web, clique no botão **CALIBRAR**.
   - O robô vai começar a calibrar. **Durante esse tempo, esfregue o sensor de linha várias vezes sobre a linha preta e sobre a fita branca para ele entender os limites de cor do piso atual.**
7. **Coloque na linha:** Posicione o sensor do robô centralizado em cima da fita preta. Note que no Dashboard Web, as "bolinhas" do QTR-8RC vão ficar pretas onde detectar linha.
8. **Inicie o modo autônomo:**
   - No Dashboard, clique em **AUTO**.
   - O carro vai começar a se mover seguindo a linha.

Se ele oscilar muito (ficar "tremendo" de um lado para o outro), não precisa reiniciar! Vá até a seção de **Ajuste de PID e Velocidade Base** no Dashboard, troque os valores (K_p, K_i, K_d) e clique em "Aplicar Ganhos" ao vivo enquanto ele anda na pista.

---

## Estrutura do Sistema de Controle

- **Core 0:** Roda o Web Server, o Wi-Fi, o ring-buffer de Logs e tarefas esporádicas de envio de dados.
- **Core 1 (RTOS Priority 7):** Loop principal de controle PID rodando a **100Hz** travados (a cada 10ms), garantindo que nada de rede atrase a matemática da curva do robô.