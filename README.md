# Sistema de Monitoramento de eficiência energética industrial

Este projeto visa o desenvolvimento de um sistema de monitoramento de eficiência energética em ambientes industriais, utilizando a plataforma ESP32 e o protocolo de comunicação MQTT. O sistema tem como objetivo monitorar o consumo de energia de máquinas industriais em tempo real, fornecendo dados essenciais para otimizar o uso de energia e melhorar a sustentabilidade operacional.

A solução foi projetada para integrar sensores de corrente elétrica (ACS712), temperatura e umidade (DHT22), e vibração (SW-420), que coletam dados em tempo real sobre as condições operacionais das máquinas. Os dados são enviados via MQTT para a nuvem, onde podem ser visualizados por meio de uma interface gráfica. Além disso, o sistema inclui um atuador (relé) que pode automatizar o controle das máquinas, desligando-as em caso de anomalias, como consumo excessivo de energia ou vibração anormal.

O projeto está alinhado com os Objetivos de Desenvolvimento Sustentável (ODS), especialmente com o ODS 9, que visa promover práticas industriais mais sustentáveis. O sistema permite que gerentes de indústria tomem decisões mais informadas para reduzir custos operacionais e minimizar o impacto ambiental, tornando os processos industriais mais eficientes e sustentáveis.

Principais Componentes:

ESP32: Microcontrolador com conectividade Wi-Fi para coletar e transmitir dados.
ACS712: Sensor de corrente elétrica para monitoramento de consumo.
DHT22: Sensor de temperatura e umidade para avaliar condições ambientais.
SW-420: Sensor de vibração para detectar falhas nas máquinas.
Atuador (Relé): Controla as máquinas, podendo desligá-las em situações críticas.
