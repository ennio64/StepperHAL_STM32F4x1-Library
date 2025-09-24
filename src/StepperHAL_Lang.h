#pragma once

// ─────────────────────────────────────────────
// Selezione lingua
// ─────────────────────────────────────────────
// Commenta la riga seguente per passare all'inglese
#define LANGUAGE_IT  // Commenta per passare all'inglese / Comment for English

// ─────────────────────────────────────────────
// Funzione di localizzazione
// ─────────────────────────────────────────────
inline String msg(const String& it, const String& en) {
#ifdef LANGUAGE_IT
  (void)en;  // evita warning unused parameter quando LANGUAGE_IT è definito
  return it;
#else
  (void)it;  // evita warning unused parameter quando LANGUAGE_IT non è definito
  return en;
#endif
}







