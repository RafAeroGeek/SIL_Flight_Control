/* ===========================================================
 * test_aircraft_loader_fcs.c
 * Cubre Aircraft_LoadFcsGains (aircraft_loader.c):
 *   - archivo válido: cada clave llena su campo de FM_Config
 *   - clave faltante: ese campo queda como lo dejó el llamador
 *   - archivo inexistente: devuelve false y no toca la config
 * =========================================================== */
#include <stdio.h>
#include <string.h>

#include "aircraft_loader.h"
#include "test_util.h"

#define TMP_JSON TEST_TMP_DIR "/fcs_gains_test.json"

static void escribir_json(const char *contenido)
{
    FILE *fp = fopen(TMP_JSON, "w");
    EXPECT_TRUE(fp != NULL);
    if (fp != NULL) {
        fputs(contenido, fp);
        fclose(fp);
    }
}

/* T1: archivo válido */
static void test_valido(void)
{
    escribir_json("{\n"
                  "  \"K_r\": 0.5,\n"
                  "  \"tau_wo_s\": 1.5,\n"
                  "  \"K_p\": 0.25,\n"
                  "  \"K_phi\": 0.75,\n"
                  "  \"K_psi\": 2.0,\n"
                  "  \"phi_max_deg\": 35.0\n"
                  "}\n");

    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    EXPECT_TRUE(Aircraft_LoadFcsGains(TMP_JSON, &cfg));
    EXPECT_FLOAT_EQ(cfg.lat.K_r, 0.5f);
    EXPECT_FLOAT_EQ(cfg.lat.tau_wo_s, 1.5f);
    EXPECT_FLOAT_EQ(cfg.lat.K_p, 0.25f);
    EXPECT_FLOAT_EQ(cfg.lat.K_phi, 0.75f);
    EXPECT_FLOAT_EQ(cfg.lat.K_psi, 2.0f);
    EXPECT_FLOAT_EQ(cfg.max_bank_deg, 35.0f);
}

/* T2: una clave faltante deja el valor previo y no afecta a las demás */
static void test_clave_faltante(void)
{
    escribir_json("{ \"K_r\": 0.5 }\n");

    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.lat.K_psi = 9.0f;
    EXPECT_TRUE(Aircraft_LoadFcsGains(TMP_JSON, &cfg));
    EXPECT_FLOAT_EQ(cfg.lat.K_r, 0.5f);
    EXPECT_FLOAT_EQ(cfg.lat.K_psi, 9.0f);
}

/* T3: archivo inexistente */
static void test_inexistente(void)
{
    FM_Config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.lat.K_r = 7.0f;
    EXPECT_FALSE(Aircraft_LoadFcsGains(TEST_TMP_DIR "/no_existe.json", &cfg));
    EXPECT_FLOAT_EQ(cfg.lat.K_r, 7.0f);
}

int main(void)
{
    test_valido();
    test_clave_faltante();
    test_inexistente();
    TEST_RETURN();
}
