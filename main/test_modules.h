#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Roda todos os testes unitários via framework Unity.
 * Usado para validar a matemática do PID e Seguidor de Linha isoladamente do hardware.
 */
void run_all_unit_tests(void);

#ifdef __cplusplus
}
#endif
