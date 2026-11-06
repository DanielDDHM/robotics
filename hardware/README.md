# Hardware: robô 2WD com ESP32-S3 e micro-ROS

Lista de materiais para levar as lições de simulação para um robô físico. A planilha editável (quantidades, preços, lojas e links) está em [`lista_compras_robo.xlsx`](lista_compras_robo.xlsx).

Preços com IVA, verificados em 06/11/2026, sem frete.

## Lista
| item | produto | loja | qtd | preço | observação |
|---|---|---|---|---|---|
| Microcontrolador | [Waveshare ESP32-S3-DEV-KIT-N8R8](https://www.botnroll.com/pt/esp32/5227-esp32-s3-placa-da-waveshare-com-esp32-s3-wroom-1-n8r8-240mhz-dual-core-usb-c-headers-soldados.html) | Bot'n Roll (PT) | 1 | 11,90 € | mesma pinagem da DevKitC-1 |
| Chassi + motores com encoder | [Chassi 2 rodas com motores JGA25 (360 CPR, 1:34)](https://funduinoshop.com/en/diy-workshop/kits/chassis/two-wheeled-balance-chassis-with-jga25-motors) | Funduino (DE) | 1 | 59,90 € | chassi de equilíbrio: precisa de roda boba |
| Driver de motor | [Adafruit TB6612](https://www.botnroll.com/pt/controladores/1957-driver-para-motor-de-passo-ate-12a-e-3-a-45v-adafruit-tb6612-.html) | Bot'n Roll (PT) | 1 | 8,95 € | |
| IMU | [Olimex MOD-MPU6050](https://www.olimex.com/Products/Modules/Sensors/MOD-MPU6050/) | Olimex (BG) | 1 | 7,95 € | |
| Baterias | [Li-Ion 18650 3,7 V 2500 mAh](https://www.botnroll.com/pt/18650/3983-pilha-li-ion-18650-3-7v-2500mah-18x65mm-26a-max.html) | Bot'n Roll (PT) | 2 | 4,40 € | |
| Suporte 2S | [Suporte 2x18650 com tampa e switch](https://www.botnroll.com/pt/18650/6156-suporte-para-2-pilhas-mr18650-com-tampa-e-switch-on-off.html) | Bot'n Roll (PT) | 1 | 2,40 € | |
| BMS 2S | [BMS 18650 2S 7,4 V 7 A](https://botnroll.com/pt/bms/4625-bms-para-protec-o-baterias-18650-2s-7-4v-7a-redondo.html) | Bot'n Roll (PT) | 1 | 3,80 € | proteção contra sobrecarga e descarga profunda |
| Regulador 5 V | [LM2596 step-down regulável](https://www.botnroll.com/pt/step-down/937-lm2596-step-down.html) | Bot'n Roll (PT) | 1 | 3,90 € | |
| Protoboard + fios | [Breadboard com kit de 140 fios](https://www.botnroll.com/pt/cabos-acessorios/75-breadboard-with-wire-kit.html) | Bot'n Roll (PT) | 1 | 7,30 € | |
| Kit de solda | [Kit de soldadura](https://www.botnroll.com/pt/kits/76-kit-soldadura-.html) | Bot'n Roll (PT) | 1 | 18,90 € | |
| Multímetro | [UNI-T UT33D+](https://www.botnroll.com/pt/multimetros/2118-multimetro-mini-digital-uni-t33d.html) | Bot'n Roll (PT) | 1 | 14,50 € | |
| Carregador 18650 | [a escolher](https://www.botnroll.com/pt/540-18650) | Bot'n Roll (PT) | 1 | ? | preço ainda não pesquisado |
| **Total** | | | | **148,30 €** | sem carregador nem fretes |

Por loja: Bot'n Roll 80,45 €, Funduino 59,90 €, Olimex 7,95 €.

### Alternativas consideradas (fora da compra)
| item | produto | preço | por que ficou de fora |
|---|---|---|---|
| ESP32-S3 | [Espressif ESP32-S3-DevKitC-1-N8R8](https://botland.store/esp32-modules/26547-esp32-s3-devkitc-1-n8r8-wifi-bluetooth-development-board-with-esp32-s3-wroom-1-chip.html) (Botland) | 22,50 € | placa oficial, mas quase o dobro do preço pela mesma pinagem |
| Motores | [Waveshare N20 12 V 200 rpm com encoder Hall](https://grobotronics.com/n20-dc-gear-motor-with-encoder-200rpm.html) (Grobotronics), 2x | 7,92 € cada | preço de quantidade; exigiria chassi à parte |
| IMU | [Adafruit BNO085](https://elecena.pl/product/29479129/adafruit-9-dof-orientation-imu-fusion-breakout-bno085-bno080) (Opencircuit) | 29,00 € | faz a fusão de sensores dentro do chip |
| IMU | [BNO055 9 eixos](https://www.botnroll.com/pt/imu-giroscopios/2959-bno055-sensor-de-orienta-o-absoluto-com-9-eixos.html) (Bot'n Roll) | 28,50 € | idem |

## Da simulação para o real
| na simulação (`robot.py`) | no robô físico |
|---|---|
| `read_encoders()`, 1000 ticks por volta | encoders Hall dos JGA25. 360 CPR na saída da caixa (cerca de 11 pulsos x 34 de redução); lendo as duas fases em quadratura (4x) dá **cerca de 1440 contagens por volta**, mais resolução que a simulação |
| `set_torque()` | TB6612 com PWM. Atenção: PWM controla **tensão**, que se traduz mais em velocidade do que em torque. O PID da lição 01 continua valendo, mas os ganhos vão ser outros |
| `read_gyro()` | MPU6050. Os parâmetros do gyro simulado na lição 03 foram baseados nele, então a calibração no boot e o EKF se aplicam diretamente |
| motor direito 7% mais fraco | vai acontecer de verdade, com outro número |
| `WHEEL_BASE`, `WHEEL_RADIUS` | medir com paquímetro ou régua; depois deixar o EKF da lição 03 refinar o wheel base |

## Pontos de atenção
1. **Faltam itens na lista.**
   - **Roda boba** (ball caster): o chassi é de equilíbrio (só 2 rodas). Sem ela o robô não fica em pé. Está na observação da planilha, mas não tem linha de compra.
   - **Jumpers dupont macho-fêmea**: o kit da protoboard tem só fios para a protoboard.
   - **Carregador 18650**: está com preço 0 na planilha.
2. **O ESP32-S3 trabalha a 3,3 V e não tolera 5 V nos pinos.** Alimente os encoders com 3,3 V (encoders Hall costumam aceitar de 3,3 a 5 V, mas confirme na ficha do motor). Se forem alimentados com 5 V, os sinais chegam a 5 V e podem danificar o ESP32 aos poucos.
3. **Ajuste o LM2596 para 5,0 V com o multímetro antes de ligar o ESP32.** Ele vem de fábrica numa tensão qualquer, e ligar sem medir é o jeito mais comum de queimar a placa logo no primeiro dia.
4. **Tensão dos motores: confirmar.** Com 2S (7,4 V nominal, 8,4 V carregado), motores de 12 V giram a cerca de 60% da velocidade nominal, o que é suficiente para um robô pequeno. Se forem motores de 6 V, a bateria ficaria acima do nominal e esquentaria os motores. Não consegui abrir a página do chassi para conferir.
5. **Corrente do TB6612.** Ele aguenta cerca de 1,2 A contínuos por canal (picos de 3,2 A). Um JGA25 travado pode passar disso. Na prática: não deixe o robô empurrando uma parede com potência máxima, e confira a corrente de stall do motor. A tensão está ok (o TB6612 aceita até 13,5 V).
6. **Encoders: use o periférico PCNT do ESP32**, que conta pulsos em hardware, em vez de interrupções. Com dois motores a 1440 contagens por volta, interrupções começam a roubar CPU e perder pulsos.
7. **Fretes: três lojas, três envios.** O MPU6050 da Olimex (7,95 €, vindo da Bulgária) pode sair mais caro no frete do que o próprio sensor. Vale ver se a Funduino ou a Bot'n Roll têm MPU6050 em stock para juntar no mesmo pedido.
8. **Baterias e BMS.** Compre 18650 de loja confiável (há muitas falsificadas com capacidade inflacionada). Com o BMS dá para carregar o pack montado usando um carregador de 8,4 V; sem um, tire as pilhas e carregue no carregador de 18650. Nunca deixe os fios da bateria soltos: um curto em 18650 é perigoso.
9. **Protoboard num robô que se move solta fios.** Ótima para testar, mas mau contato gera bugs intermitentes difíceis de achar (o tipo de bug que o multímetro resolve). Quando o circuito funcionar, vale passar para uma placa perfurada soldada.
10. **BNO085 e BNO055 ficaram de fora, e com razão.** Eles entregam a orientação já fundida, o que é ótimo para um produto, mas pula exatamente o que a lição 03 ensina. Fazer o próprio EKF com o MPU6050 ensina mais. Ficam como upgrade futuro.
