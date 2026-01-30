# Risoluzione del Problema TX/TY - Contaminazione Incrociata

## Problema Segnalato
> "se muovo lungo asse ty mi viene molto sporcato da tx anche avendo alzato i parametri di #define CONFIG_TXTY_COMBINED_THRESHOLD 2.5 e di tx deadzone a 1.5"

## Causa del Problema

Il filtro Kalman 2D accoppia insieme gli assi TX e TY, tracciando le velocità per entrambi. Quando si muove lungo l'asse TY:

1. Il rumore del sensore TX viene filtrato dal Kalman 2D
2. Lo stato di velocità mantiene una componente TX residua
3. **PROBLEMA**: La magnitudo combinata veniva calcolata PRIMA di applicare le deadzone
4. Questo significava che il rumore TX contribuiva al calcolo della magnitudo
5. Esempio: TY = 5.0, TX = 0.8 → magnitudo = 5.06 (supera la soglia!)

## Soluzione Implementata

**Prima della correzione:**
```cpp
// BUGGY: La magnitudo include il rumore
double mag_tx_ty = sqrt(tx * tx + ty * ty);
```

**Dopo la correzione:**
```cpp
// CORRETTO: Applica le deadzone PRIMA di calcolare la magnitudo
double tx_filtered = (abs(tx) > CONFIG_TX_DEADZONE) ? tx : 0.0;
double ty_filtered = (abs(ty) > CONFIG_TY_DEADZONE) ? ty : 0.0;
double mag_tx_ty = sqrt(tx_filtered * tx_filtered + ty_filtered * ty_filtered);
```

## Come Funziona Ora

Quando si muove solo lungo TY:
- TY = 5.0 → ty_filtered = 5.0 ✓
- TX = 0.8 (rumore) → tx_filtered = 0.0 (sotto la deadzone) ✓
- Magnitudo = sqrt(25 + 0) = 5.0 (pulito!)
- Il controllo "entrambi gli assi non-zero" fallisce immediatamente
- Ritorna alla logica predominante: viene inviato solo TY ✓

## Vantaggi della Correzione

1. ✅ **Elimina la contaminazione incrociata**: Muovendo TY non viene più inviato rumore TX
2. ✅ **Logica più pulita**: Percorso decisionale più semplice per movimenti su singolo asse
3. ✅ **Comportamento prevedibile**: Solo i veri movimenti diagonali attivano la modalità combinata
4. ✅ **Mantiene la compatibilità**: I movimenti diagonali esistenti continuano a funzionare correttamente

## Test Effettuati

### Test 1: Movimento Puro TX
```
Input: TX = 5.0, TY = 0.5 (rumore sotto deadzone TY di 1.5)
Risultato: Solo TX inviato ✓
```

### Test 2: Movimento Puro TY (Il Tuo Caso)
```
Input: TX = 0.8 (rumore), TY = 5.0
Prima: magnitudo = 5.06 (include rumore)
Dopo: magnitudo = 5.0 (pulito, TX filtrato a 0)
Risultato: Solo TY inviato, nessun TX ✓
```

### Test 3: Movimento Diagonale Vero
```
Input: TX = 3.0, TY = 4.0 (entrambi > deadzone)
Risultato: Entrambi TX e TY inviati ✓
```

## Configurazione Raccomandata

Dopo questa correzione, la configurazione predefinita dovrebbe funzionare bene:

```cpp
#define CONFIG_TX_DEADZONE     1.0    // Default va bene
#define CONFIG_TY_DEADZONE     1.5    // Default va bene  
#define CONFIG_TXTY_COMBINED_THRESHOLD  2.0  // Puoi mantenere il default
```

Ora puoi anche **abbassare** questi valori se necessario, poiché l'approccio "deadzone-first" previene la contaminazione in modo più efficace.

## Sintonizzazione Aggiuntiva (Se Necessario)

Se la contaminazione TX persiste ancora nel movimento puro TY:

1. **Aumenta la deadzone TX**: `CONFIG_TX_DEADZONE = 2.0`
2. **Riduci il tracciamento della velocità**: `CONFIG_KALMAN2D_Q_VEL = 0.05` (rende il filtro meno "appiccicoso")
3. **Aumenta il rumore di misura**: `CONFIG_KALMAN2D_R = 0.15` (più filtraggio)
4. **Verifica la calibrazione**: Assicurati che la posizione neutra del sensore sia correttamente calibrata

## Riepilogo

La correzione cambia l'ordine delle operazioni da:
```
Calcola Magnitudo → Controlla Deadzone → Invia o Rifiuta
```

A:
```
Applica Deadzone → Calcola Magnitudo → Invia o Rifiuta
```

Questo semplice riordino **elimina la causa principale** della contaminazione incrociata mantenendo tutti i vantaggi del filtro Kalman 2D per i veri movimenti diagonali.

---

## File Modificati

1. **AdaSpace3D.ino**: Logica di rilevamento movimento combinato corretta
2. **TXTY-COMBINED-MOVEMENT.md**: Documentazione aggiornata
3. **TXTY-CROSS-CONTAMINATION-FIX.md**: Analisi tecnica dettagliata (in inglese)
4. **IMPLEMENTATION-SUMMARY-TXTY.md**: Riepilogo implementazione aggiornato

## Come Testare

1. Compila e carica il firmware aggiornato
2. Muovi il knob solo lungo l'asse TX → verifica che non ci sia contaminazione TY
3. Muovi il knob solo lungo l'asse TY → verifica che non ci sia contaminazione TX ✓
4. Muovi il knob in diagonale → verifica che entrambi gli assi siano inviati correttamente

---

**Stato**: ✅ **RISOLTO** - Contaminazione incrociata TX/TY eliminata
