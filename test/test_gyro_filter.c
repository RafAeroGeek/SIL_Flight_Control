/* ===========================================================
 * test_gyro_filter.c
 * Cubre gyro_filter.c y lpf1.c:
 *   - respuesta al escalón igual a la solución exacta de 1er orden
 *   - ganancia y fase en la frecuencia de corte
 *   - fc = 0 sin filtro; fc > máximo limitado; ejes independientes
 *   - la primera muestra inicializa el estado
 * =========================================================== */
#include <math.h>

#include "gyro_filter.h"
#include "test_util.h"

#define DT_S 0.005
#define PI_D 3.14159265358979323846

static bool cerca(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

/* T1: la primera muestra inicializa y una entrada constante no se mueve */
static void test_primera_muestra(void)
{
    const float fc[3] = { 20.0f, 20.0f, 20.0f };
    const double u[3] = { 1.0, -2.0, 3.0 };
    double y[3];
    GyroFilter f;

    GyroFilter_Init(&f, fc);
    for (int k = 0; k < 10; ++k) {
        GyroFilter_Update(&f, u, DT_S, y);
        EXPECT_TRUE(cerca(y[0], 1.0, 1e-12));
        EXPECT_TRUE(cerca(y[1], -2.0, 1e-12));
        EXPECT_TRUE(cerca(y[2], 3.0, 1e-12));
    }
}

/* T2: escalón a 5 Hz contra 1 - exp(-t/tau) */
static void test_escalon(void)
{
    const float fc[3] = { 5.0f, 5.0f, 5.0f };
    const double cero[3] = { 0.0, 0.0, 0.0 };
    const double uno[3]  = { 1.0, 1.0, 1.0 };
    const double tau = 1.0 / (2.0 * PI_D * 5.0);
    double y[3];
    GyroFilter f;

    GyroFilter_Init(&f, fc);
    GyroFilter_Update(&f, cero, DT_S, y);
    for (int k = 1; k <= 100; ++k) {
        GyroFilter_Update(&f, uno, DT_S, y);
        EXPECT_TRUE(cerca(y[0], 1.0 - exp(-k * DT_S / tau), 1e-4));
    }
}

/* T3: senoidal en fc: ganancia 1/sqrt(2). La fase es de unos -45° más medio
   paso, porque la salida ya incluye la muestra actual */
static void test_corte(void)
{
    const float fc_hz = 5.0f;
    const float fc[3] = { fc_hz, fc_hz, fc_hz };
    const double w = 2.0 * PI_D * fc_hz;
    GyroFilter f;
    GyroFilter_Init(&f, fc);

    double a = 0.0, b = 0.0;
    int n = 0;
    for (int k = 0; k < 2400; ++k) {   // 12 s; 10 s enteros de medición
        const double t = k * DT_S;
        const double s = sin(w * t);
        const double u[3] = { s, s, s };
        double y[3];
        GyroFilter_Update(&f, u, DT_S, y);
        if (k >= 400) {
            a += y[0] * sin(w * t);
            b += y[0] * cos(w * t);
            ++n;
        }
    }
    a *= 2.0 / n;
    b *= 2.0 / n;
    const double fase_deg = atan2(b, a) * 180.0 / PI_D;
    EXPECT_TRUE(cerca(hypot(a, b), 1.0 / sqrt(2.0), 0.03));
    EXPECT_TRUE(cerca(fase_deg, -45.0 + 0.5 * DT_S * w * 180.0 / PI_D, 1.0));
}

/* T4: fc = 0 no filtra, fc > máximo se limita, y cada eje usa su corte */
static void test_configuracion(void)
{
    const float fc[3] = { 0.0f, 100.0f, 5.0f };
    const double cero[3] = { 0.0, 0.0, 0.0 };
    const double uno[3]  = { 1.0, 1.0, 1.0 };
    double y[3];
    GyroFilter f;

    GyroFilter_Init(&f, fc);
    EXPECT_FLOAT_EQ(f.eje[0].fc_hz, 0.0f);
    EXPECT_FLOAT_EQ(f.eje[1].fc_hz, LPF1_FC_MAX_HZ);
    EXPECT_FLOAT_EQ(f.eje[2].fc_hz, 5.0f);

    GyroFilter_Update(&f, cero, DT_S, y);
    GyroFilter_Update(&f, uno, DT_S, y);
    EXPECT_TRUE(y[0] == 1.0);
    EXPECT_TRUE(y[1] > y[2]);
    EXPECT_TRUE(y[2] > 0.0 && y[2] < 1.0);
}

int main(void)
{
    test_primera_muestra();
    test_escalon();
    test_corte();
    test_configuracion();
    TEST_RETURN();
}
