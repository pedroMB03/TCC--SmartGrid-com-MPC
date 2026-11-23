# Model Predictive Control (MPC) e Gestão de Cargas (HEMS) na Microrrede Residencial
## Documento Técnico e Metodológico para o Trabalho de Conclusão de Curso (TCC)

---

## 1. Visão Geral: Por que a Ação do MPC é Fundamentalmente Diferente do RBC?

Para responder com precisão acadêmica na sua banca:

### 1.1 Controle Baseado em Regras (RBC - *Rule-Based Control*)
* **Natureza:** Reativo, determinístico e de **horizonte míope** ($H = 1$).
* **Como toma decisões:** Em cada instante $t$, o RBC olha apenas para o balanço presente:
  $$\Delta P(t) = P_{\text{solar}}(t) - P_{\text{carga}}(t)$$
  * Se $\Delta P(t) > 0$ (sobra solar): Tenta carregar a bateria; se a bateria estiver cheia, exporta para a rede.
  * Se $\Delta P(t) < 0$ (déficit de energia): Descarrega a bateria para alimentar a casa; se a bateria esgotar, compra da rede.
* **A falha econômica do RBC:** O RBC não tem **memória futura** nem conhecimento de tarifas. Se às 14:00 h passar uma nuvem ou ligar um forno, o RBC descarrega a bateria imediatamente a uma tarifa barata ($\text{R\$}~0{,}75/\text{kWh}$). Quando chega o horário de ponta (**18:00 h às 21:00 h**), onde a tarifa salta para **$\text{R\$}~1{,}20/\text{kWh}$**, a bateria já está descarregada, forçando a residência a comprar energia caríssima da rede.

### 1.2 Controle Preditivo Baseado em Modelo (MPC - *Model Predictive Control*)
* **Natureza:** Proativo, antecipatório e de **horizonte deslizante** ($H = 24\text{ horas}$).
* **Como toma decisões:** Em cada instante $t$, o MPC resolve um problema de **Otimização Convexa** (Programação Linear / Quadrática via CVXPY) minimizando uma função de custo ao longo das próximas 24 horas à frente.
* **A vantagem econômica do MPC:** O MPC sabe que a tarifa ficará cara às 18:00 h e sabe com antecedência a previsão de sol. Portanto, ele:
  1. **Poupa a bateria** durante a tarde (mesmo que haja déficit temporário), mantendo-a com carga máxima.
  2. **Descarrega a bateria estrategicamente** exatamente entre 18:00 h e 21:00 h, zerando a importação da rede no horário mais caro.
  3. Aplica o princípio do **horizonte deslizante** (*Receding Horizon*): calcula as ações para $t, t+1, \dots, t+23$, mas **aplica apenas a primeira ação $u(t)$**. No passo seguinte ($t+1$), recebe o novo estado real e reotimiza.

---

## 2. Definição Física dos Sinais: Entradas ($u$), Estados ($x$) e Saídas ($y$)

Na teoria clássica de controle e no framework `pymgrid`:

### 2.1 Vetor de Ações de Controle: $u(t)$
O sinal $u(t)$ é a decisão que o controlador envia para os atuadores da microrrede a cada hora:
* **`u['battery']` (kW ou kWh/passo):**
  * Potência solicitada à bateria.
  * No `pymgrid`, a ação normalizada $[-1, 1]$ ou física representa:
    * Valor **positivo ($> 0$):** Comando de **Carga** (armazenar energia na bateria).
    * Valor **negativo ($< 0$):** Comando de **Descarga** (fornecer energia para a casa).
    * Valor **nulo ($= 0$):** Bateria em espera (*idle* / flutuação).
* **`u['grid']` (kW ou kWh/passo):**
  * Potência solicitada de troca com a rede da concessionária.
  * Valor **positivo:** Importação de energia (compra da rede).
  * Valor **negativo:** Exportação de energia (injeção de excedente solar).

### 2.2 Vetor de Estados Físicos Internos: $x(t)$
O estado representa a memória física do sistema dinâmico:
* **$SoC(t)$ (State of Charge - Estado de Carga da Bateria):**
  $$SoC(t) = \frac{E_{\text{bateria}}(t)}{E_{\text{max}}} \in [SoC_{\min}, SoC_{\max}]$$
  No seu código: $SoC_{\min} = 10\%$ ($0{,}5\text{ kWh}$) e $SoC_{\max} = 100\%$ ($5{,}0\text{ kWh}$).
* **Equação de Transição de Estado da Bateria:**
  $$E_{\text{bat}}(t+1) = E_{\text{bat}}(t) + \left( \eta_{\text{chg}} \cdot P_{\text{chg}}(t) - \frac{P_{\text{dis}}(t)}{\eta_{\text{dis}}} \right) \Delta t$$
  onde $\eta_{\text{chg}} = \eta_{\text{dis}} = \sqrt{0{,}95} \approx 0{,}9747$ (eficiência global de ciclo $= 95\%$).

### 2.3 Vetor de Saídas e Medições: $y(t)$
O vetor $y(t)$ é a observação retornada pelo ambiente (`casa_mpc.step()`) após aplicar a ação $u(t)$:
* **`y['load']`:** Consumo real medido da residência na hora $t$ (em kWh).
* **`y['pv']`:** Geração solar fotovoltaica real na hora $t$ (em kWh).
* **`y['battery']`:** Vetor físico composto por $[SoC(t), P_{\text{bat\_real}}(t)]$.
* **`y['grid']`:** Vetor físico contendo $[P_{\text{import}}(t), P_{\text{export}}(t), \text{preço}, CO_2]$.

---

## 3. Formulação Matemática das Regras e Restrições do MPC

O MPC não usa "regras if/else" humanas. Em vez disso, as regras são traduzidas para **Função Objetivo** e **Restrições de Desigualdade/Igualdade**:

### 3.1 Função Objetivo (Minimização de Custo Financeiro)
A cada passo $t$, o MPC resolve:
$$\min_{P_{\text{imp}}, P_{\text{exp}}, P_{\text{chg}}, P_{\text{dis}}} \sum_{k=0}^{H-1} \Big( c_{\text{imp}}(t+k) \cdot P_{\text{imp}}(t+k) - c_{\text{exp}}(t+k) \cdot P_{\text{exp}}(t+k) + c_{\text{deg}} \cdot P_{\text{bat}}(t+k) \Big)$$

* $c_{\text{imp}}(t+k)$: Tarifa de importação no instante futuro (R$ 1,20 na ponta, R$ 0,75 fora).
* $c_{\text{exp}}(t+k)$: Tarifa de exportação (valor de venda do excedente solar).
* $c_{\text{deg}}$: Custo de degradação da bateria (configurado como $0{,}0$ no seu TCC).

### 3.2 Restrições Operacionais (As "Regras" Físicas)
1. **Balanço Instantâneo de Potência (1ª Lei de Kirchhoff):**
   $$P_{\text{solar}}(t+k) + P_{\text{imp}}(t+k) + P_{\text{dis}}(t+k) = P_{\text{carga}}(t+k) + P_{\text{exp}}(t+k) + P_{\text{chg}}(t+k)$$
2. **Capacidade do Inversor e Conexão da Rede:**
   $$0 \le P_{\text{imp}}(t+k) \le P_{\text{imp}}^{\max} \quad (5{,}0\text{ kW})$$
   $$0 \le P_{\text{exp}}(t+k) \le P_{\text{exp}}^{\max} \quad (3{,}0\text{ kW})$$
3. **Limites de Potência da Bateria:**
   $$0 \le P_{\text{chg}}(t+k) \le P_{\text{chg}}^{\max} \quad (2{,}5\text{ kW})$$
   $$0 \le P_{\text{dis}}(t+k) \le P_{\text{dis}}^{\max} \quad (2{,}5\text{ kW})$$
4. **Limites de Energia Armazenada (SoC):**
   $$E_{\min} \le E_{\text{bat}}(t+k) \le E_{\max} \quad (0{,}5\text{ kWh} \le E_{\text{bat}} \le 5{,}0\text{ kWh})$$

---

## 4. Explicação Linha por Linha do Código (Da Célula 39 em Diante)

### Célula 39: Definição do Horizonte Temporal do MPC
```python
HORAS_MPC = 24
h = 24
n = HORAS_MPC + h + 1
horas_mpc = np.arange(n) % HORAS_MPC
```
* `HORAS_MPC = 24`: Horizonte de controle de 1 dia de simulação (24 passos de 1 hora).
* `h = 24`: Horizonte de predição futuro que o MPC precisa enxergar além do passo atual.
* `n = HORAS_MPC + h + 1 = 49`: Cria uma série temporal de 49 horas.
  * **Por que precisa de 49 horas?** Quando a simulação estiver na hora $t = 23$, o MPC precisa enxergar 24 horas à frente ($t=23$ até $t=47$). Sem essas horas adicionais, o solver do MPC geraria erro de *IndexOutOfBounds*.
* `np.arange(n) % HORAS_MPC`: Faz o vetor de horas repetir ciclicamente de $0$ a $23$ (dia seguinte periódico).

---

### Célula 40: Série Temporal da Carga para o MPC
```python
carga_kwh_mpc = perfil_residencia[horas_mpc]
```
* Replica o perfil de carga de 24 horas ao longo das 49 horas do horizonte estendido do MPC.

---

### Célula 41: Geração Solar e Matriz Tarifária
```python
formula_mpc = np.clip(np.sin((horas_mpc - 6) * np.pi / 12), 0, None)
energia_dia = kwp * horas_sol_pleno * pr
geracao_solar_mpc = energia_dia * formula_mpc / formula_mpc[:HORAS_MPC].sum()
```
* Modela a curva senoidal de irradiação solar começando às 06:00 h e terminando às 18:00 h.
* `formula_mpc / formula_mpc[:HORAS_MPC].sum()` normaliza a integral para garantir que a energia gerada no dia seja exatamente igual ao dimensionamento fotovoltaico ($E = \text{kWp} \times \text{HSP} \times \text{PR}$).

```python
tarifa_import_mpc = np.where(
    (horas_mpc >= 18) & (horas_mpc < 21),
    1.20,
    0.75
)
tarifa_export_mpc = tarifa_import_mpc * 0.75

grid_ts_mpc = pd.DataFrame({
    "grid_price_import": tarifa_import_mpc,
    "grid_price_export": tarifa_export_mpc,
    "grid_co2": np.full(len(horas_mpc), 0.4)
})
```
* **Tarifa Branca:** Aplica a tarifa de ponta (R$ 1,20) das 18h às 21h e fora de ponta (R$ 0,75) no restante do dia.
* **Tarifa de Exportação:** 75% da tarifa de importação (representando a remuneração com abatimento de custos de distribuição segundo a Lei 14.300 / GD).
* Cria o `DataFrame` com as colunas exigidas pelo módulo de rede do `pymgrid` (`grid_price_import`, `grid_price_export`, `grid_co2`).

---

### Célula 42: Instanciação da Microrrede MPC
```python
def criar_casa(forecaster=None, horizonte = 0):
    kw = dict(forecaster=forecaster, forecast_horizon=horizonte) if forecaster else {}

    bateria = BatteryModule(
        min_capacity = 0.5,
        max_capacity = 5,
        max_charge = 2.5,
        max_discharge = 2.5,
        efficiency = 0.95,
        battery_cost_cycle = 0.0,
        init_soc = 0.2)

    carga_mpc = LoadModule(time_series=carga_kwh_mpc, **kw)
    solar_mpc = RenewableModule(time_series=geracao_solar_mpc, **kw)
    rede_mpc = GridModule(max_import=5, max_export=3, time_series=grid_ts_mpc, **kw)

    return Microgrid([bateria, ("pv", solar_mpc), carga_mpc, rede_mpc])
```
* `forecaster="oracle"`: Configura um preditor perfeito (ideal), onde o MPC tem a previsão exata de carga e sol das próximas 24h (padrão em estudos comparativos de limite superior de desempenho).
* Parâmetros físicos:
  * Bateria de Lítio: capacidade útil de $0{,}5\text{ a }5{,}0\text{ kWh}$, potência máxima de carga e descarga de $2{,}5\text{ kW}$, estado inicial de $20\%$ ($1{,}0\text{ kWh}$).
  * Conexão da concessionária: disjuntor de importação de $5\text{ kW}$ e limite de injeção de $3\text{ kW}$.

---

### Células 44 a 46: Identificação e Correção do Bug do `pymgrid` (`CustomMPC`)
```python
class CustomMPC(ModelPredictiveControl):
    def __init__(self, microgrid, solver=None):
        super().__init__(microgrid, solver=solver)
        if not self.has_genset:
            self.costs = cp.Parameter(7 * self.horizon)
            parameters = self._parse_microgrid()
            self.problem = self._create_problem(*parameters)

    def _set_parameters(self, load_vector, pv_vector, grid_vector, import_price, export_price, ...):
        super()._set_parameters(...)
        if not self.has_genset:
            self.costs.value[1::7] = -export_price.reshape(-1)
```
* **O Bug Original do `pymgrid`:**
  Na biblioteca `pymgrid`, a função objetivo era declarada como `costs = cp.Parameter(7 * horizon, nonneg=True)`. Como o solver minimiza a função, um preço de exportação positivo era somado ao custo total:
  $$\min \dots + (\text{export\_price} \times P_{\text{export}})$$
  O solver interpretava que vender energia era um **gasto**, e por isso o MPC nunca vendia energia e cortava (*curtailed*) o excedente solar.
* **A Correção Científica (`CustomMPC`):**
  1. Remove a restrição `nonneg=True` permitindo coeficientes negativos.
  2. Define `self.costs.value[1::7] = -export_price`, transformando a exportação em **receita/crédito** (valor que diminui a função objetivo).

---

### Célula 47: O Loop de Controle em Tempo Real do MPC
```python
casa_mpc = criar_casa(forecaster="oracle", horizonte=HORAS_MPC)
mpc = CustomMPC(casa_mpc)

vetor_U_mpc = []
vetor_Y_mpc = []
vetor_recompensa_mpc = []

vetor_Y_mpc.append(casa_mpc.reset())

for t in range(HORAS):
    U = mpc.get_action()
    Y, recompensa, done, info = casa_mpc.step(U, normalized=False)

    vetor_U_mpc.append(U)
    vetor_Y_mpc.append(Y)
    vetor_recompensa_mpc.append(recompensa)

    if done:
        break

log_mpc = casa_mpc.get_log(as_frame=True)
```
* `U = mpc.get_action()`: O MPC lê o estado atual da casa, monta as matrizes do problema de otimização, invoca o solver CVXPY, resolve a trajetória de 24 horas futuras e extrai a ação ótima para o passo presente $t$.
* `casa_mpc.step(U, normalized=False)`: Aplica a ação física nos módulos da microrrede e atualiza a simulação dinâmica para o próximo estado $t+1$.
* Salva os sinais de controle ($U$) e saídas ($Y$) a cada passo para análise e geração dos gráficos.

---

### Célula 50: Desnormalização das Observações para Unidades Físicas
```python
for observacao in vetor_Y_mpc:
    ...
    lista_desnorm.append(modulos[nome].from_normalized(val, obs=True))
```
* O ambiente Gym do `pymgrid` armazena medições internas normalizadas no intervalo $[0, 1]$ ou $[-1, 1]$ para compatibilidade com algoritmos de Aprendizado por Reforço (RL).
* Essa célula executa o método `from_normalized()`, convertendo os vetores de volta para grandezas de engenharia com significado físico: **kWh, kW e porcentagem (%)**.

---

### Células 52 e 53: Gráficos de Sinais $u(t)$ e $y(t)$
* **Célula 52 (Sinais de Controle $u(t)$):**
  * Plota a curva de potência solicitada à bateria e à rede.
  * **O que comprova:** Mostra que durante as horas solares a bateria recebe potência positiva (carregando), e exatamente às 18:00 h a potência vai para negativa máxima (descarregando para suportar a ponta), mantendo a compra da rede em zero ou no mínimo possível.
* **Célula 53 (Sinais de Saída $y(t)$):**
  * Plota a geração solar medida e o consumo da carga ao longo do dia, validando o equilíbrio de potência.

---

### Célula 54 e 55: Tabela de Balanço Energético Horário
```python
resumo_mpc = pd.DataFrame({
    "horas": np.arange(1, len(log_mpc) + 1),
    "consumo_kwh": log_mpc[("load", 0, "load_met")].round(2).values,
    "solar_kwh": log_mpc[("pv", 0, "renewable_current")].abs().round(2).values,
    "soc_bateria_%": (log_mpc[("battery", 0, "soc")] * 100).round(0).values,
    "carga_bateria_kwh": log_mpc[("battery", 0, "charge_amount")].round(2).values,
    "compra_rede_kwh": log_mpc[("grid", 0, "grid_import")].round(2).values,
    "venda_rede_kwh": log_mpc[("grid", 0, "grid_export")].round(2).values,
    "custo_hora_R$": (-log_mpc[("balance", 0, "reward")]).round(2).values,
})
```
* Consolida os logs internos da microrrede hora a hora:
  * Mostra a evolução do SoC (inicia em 20%, sobe até 100% ao meio-dia e desce até 10% durante a ponta).
  * Exibe o custo horário em Reais, demonstrando ausência de custos elevados entre 18h e 21h.

---

### Células 56 a 59: Comparativo Direto RBC vs MPC
* Compara as faturas de energia finais geradas pelas duas estratégias sob **as mesmas condições de sol e carga**.
* **Fórmula do Custo Líquido Diário:**
  $$\text{Custo Total (R\$)} = \sum_{t=1}^{24} \Big( P_{\text{import}}(t) \times c_{\text{import}}(t) - P_{\text{export}}(t) \times c_{\text{export}}(t) \Big)$$
* O recálculo manual nas células 57 e 58 valida que a economia do MPC não é ilusão de métrica ou taxa de degradação, mas sim redução real da conta de energia da concessionária.

---

### Células 60 a 69: A Camada Superior de HEMS (Gestão de Cargas BL e IL)

#### 1. Separação Conceitual (Células 61 e 62)
* **BL (Base Load - Cargas Inflexíveis):** Geladeira, iluminação, eletrônicos em standby, chuveiro, televisão. O morador não aceita ter o uso dessas cargas remanejado pelo sistema.
* **IL (Interruptible/Shiftable Loads - Cargas Flexíveis):** Máquina de lavar, robô aspirador e ar-condicionado. Podem ser deslocadas temporalmente para maximizar o autoconsumo solar e fugir da tarifa de ponta.

#### 2. O Algoritmo de Otimização `otimizar_cargas` (Célula 65)
* Realiza uma busca ótima coordenada em dois níveis:
  1. **Máquina de Lavar:** Deslocada para o bloco contínuo de 2 horas de maior excedente solar dentro da janela permitida (08h às 16h).
  2. **Robô Aspirador:** Alocado sequencialmente após a máquina de lavar, aproveitando a sobra solar restante e evitando sobrecarga.
  3. **Ar-Condicionado:** Deslocado das 18h–22h (horário de ponta) para as 14h–18h (pré-resfriamento térmico com sol abundante e tarifa barata).
* **Conservação Estrita de Energia:** A função preserva exatamente a mesma quantidade total de kWh consumidos pela residência no dia, comprovando que a economia decorre do **gerenciamento inteligente** e não de cortes no padrão de vida do usuário.

#### 3. Simulação Final dos 3 Níveis (Célula 69)
A microrrede é instanciada e simulada com a nova carga $P_{\text{hems}}(t)$ sob o MPC:
1. **Nível 1 - RBC Tradicional (Sem HEMS):** Controle reativo clássico.
2. **Nível 2 - MPC Convencional (Sem HEMS):** Controle preditivo otimizando apenas bateria e rede.
3. **Nível 3 - MPC + HEMS (Controle Preditivo com Gestão da Demanda):** Otimização conjunta dos fluxos de potência e do agendamento dos equipamentos.

---

## 5. O que os Gráficos de Sinais Comprovam para a sua Banca?

1. **Gráfico de Ações de Controle $u(t)$:**
   * **Comprovação:** Comprova a inteligência antecipatória do MPC. Diferente de um controlador liga/desliga, o sinal da bateria modula suavemente a potência e preserva a energia armazenada até o momento exato de maior retorno financeiro (18:00 h).
2. **Gráfico de Carga Original vs Carga HEMS:**
   * **Comprovação:** Comprova o efeito de *Peak Clipping* (rebaixamento do pico das 18h às 21h) e *Valley Filling* / *Solar Load Shifting* (preenchimento do vale de consumo diurno para coincidir com a geração fotovoltaica).
3. **Gráfico de Balanço Energético e SoC:**
   * **Comprovação:** Comprova o cumprimento irrestrito das restrições de segurança do sistema: o SoC nunca viola o limite de 10% (preservando a vida útil da bateria de Lítio) e a rede nunca ultrapassa o limite físico do disjuntor de entrada.
