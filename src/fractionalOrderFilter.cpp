#include "fractionalOrderFilter.h"
#include <cmath>
#include <cstring> // Para memmove

FractionalOrderFilter::FractionalOrderFilter(double orden, double periodo, double tipo) {
     if (orden < MIN_ORDER || orden > MAX_ORDER || tipo < MIN_TYPE || tipo > MAX_TYPE) {
        return;
    }
    gC.orden = orden;
    gC.periodo = periodo;
    gC.tipo = tipo;
}

void FractionalOrderFilter::gaussJordan(double* A, double* b, int N) {
    for (int i = 0; i < N; i++) {
        double pivot = A[i * N + i];
        for (int j = i; j < N; j++) {
            A[i * N + j] /= pivot;
        }
        b[i] /= pivot;
        for (int j = 0; j < N; j++) {
            if (j != i) {
                double factor = A[j * N + i];
                for (int k = i; k < N; k++) {
                    A[j * N + k] -= factor * A[i * N + k];
                }
                b[j] -= factor * b[i];
            }
        }
    }
}


void FractionalOrderFilter::respuestaImpulso(int cantidad, double* resultado) {
    if (cantidad <= 0) {
        return;
    }

    double scale = pow(1.0 / gC.periodo, gC.orden);
    double k1 = pow(1.0 / gC.tipo, gC.orden);
    double k2 = (1.0 - gC.tipo) / gC.tipo;
    double DD[cantidad];
    double k2_power[cantidad];
    double EE = 1;
    double EE1 = 1;
    DD[0] = 1;
    k2_power[0] = 1.0;

    for (int n = 0; n < cantidad; n++) {
        double sum = 0.0;
        if (gC.tipo == 1.0) {
            if (n > 0) {
                EE *= (1 - (1 + gC.orden) / n);
            }
            sum = EE;
        } else {
            if (n > 0) {
            EE *= (1 - (1 + gC.orden) / n);
            EE1 *= (1 - (1 - gC.orden) / n);
            int sign = (n % 2 == 0) ? 1 : -1;
            DD[n] = sign * EE1;
            k2_power[n] = k2_power[n - 1] * k2;
            }
            for (int k = 0; k <= n; k++) {
                double term = k1 * k2_power[n - k] * EE * DD[n - k];
                sum += term;
            }
        }
        resultado[n] = scale * sum;
    }
}

void FractionalOrderFilter::numPade(double*h, double* num,double ordenNum,double* den,double ordenDen){
    for (int n = 0; n <= ordenNum; n++) {
        int limite = (n < ordenDen) ? n : ordenDen;
        for (int k = 0; k <= limite; k++) {
            num[n] += den[k] * h[n - k];
        }
    }
}

void FractionalOrderFilter::Pade(double* h, double* num, int ordenNum, double* den, int ordenDen) {
    double dH[ordenDen * ordenDen] = {0.0};
    for (int i = 0; i < ordenDen; i++) {
        for (int j = 0; j < ordenDen; j++) {
            dH[ordenDen * i + j] = h[ordenNum + i - j];
        }
    }
    for (int i = 0; i < ordenDen; i++) {
        den[i] = -h[ordenNum + 1 + i];
    }
    gaussJordan(dH,den,ordenDen);
    memmove(&den[1], den, ordenDen * sizeof(double));
    den[0] = 1;
    numPade(h,num,ordenNum,den,ordenDen);
}

void FractionalOrderFilter::Pade(double* num, int ordenNum, double* den, int ordenDen) {
    double h[ordenNum + ordenDen + 1];
    respuestaImpulso(ordenNum + ordenDen + 1, h);
    ponerCeros(num,ordenNum,den,ordenDen);
    Pade(h, num, ordenNum, den, ordenDen);
}

void FractionalOrderFilter::calcularMatriz(const MatrixParams& params) {
    for (int i = 0; i < params.filas; i++) {
        for (int j = 0; j <= i; j++) {
            double sumMatrix = 0.0;
            double sumResult = 0.0;
            for (int k = params.inicio; k < params.fin; k++) {
                double val_i = (k - i >= 0) ? params.g_o_h[k - i] : 0.0;
                double val_j = (k - j >= 0) ? params.g_o_h[k - j] : 0.0;
                sumMatrix += val_i * val_j;
                if (j == 0) {
                    sumResult += val_i * (params.positivo ? params.h[k] : params.h[k + 1]);
                }
            }
            params.matriz[params.filas * i + j] = sumMatrix;
            if (i != j) {
                params.matriz[params.filas * j + i] = sumMatrix;
            }
            if (j == 0) {
                params.result[i] = (params.positivo ? sumResult : -sumResult);
            }
        }
    }
}

void FractionalOrderFilter::Prony(double* h, double* num, int ordenNum, double* den, int ordenDen, int cantidad, bool flag) {
    double HTH[ordenDen * ordenDen] = {0.0};
    MatrixParams params = {HTH, h, h, ordenDen, ordenNum, cantidad - 1, den, false};
    calcularMatriz(params);
    gaussJordan(HTH, den, ordenDen);
    memmove(&den[1], den, ordenDen * sizeof(double));
    den[0] = 1;
    if (flag == true) {
        numPade(h, num, ordenNum, den, ordenDen);
    }
}

void FractionalOrderFilter::Prony(double* num, int ordenNum, double* den, int ordenDen, int cantidad) {
    if (cantidad<=ordenNum+ordenDen){
        ponerCeros(num,ordenNum,den,ordenDen);
        return;
    }else if(cantidad == ordenNum+ordenDen +1){
        Pade(num,ordenNum,den,ordenDen);
    }else{
        ponerCeros(num,ordenNum,den,ordenDen);
        double h[cantidad]={0.0};
        respuestaImpulso(cantidad, h);
        Prony(h, num, ordenNum, den, ordenDen,cantidad,true);
    }
}

void FractionalOrderFilter::Shank(double* h, double* num, int ordenNum, double* den, int ordenDen, int cantidad) {
    double g[cantidad] = {0};
    g[0] = 1;
    for (int n = 1; n < cantidad; n++) {
        int limite = (n < ordenDen) ? n : ordenDen;
        double sum = 0.0;
        for (int k = 1; k <= limite; k++) {
            sum -= den[k] * g[n - k];
        }
        g[n] = sum;
    }
    double GTG[(ordenNum + 1) * (ordenNum + 1)] = {0.0};
    MatrixParams params = {GTG, h, g, ordenNum + 1, 0, cantidad, num, true};
    calcularMatriz(params);
    gaussJordan(GTG, num, ordenNum + 1);
}


void FractionalOrderFilter::Shank(double* num, int ordenNum, double* den, int ordenDen, int cantidad) {
    if (cantidad<=ordenNum+ordenDen){
        ponerCeros(num,ordenNum,den,ordenDen);
        return;
    }else if(cantidad == ordenNum+ordenDen +1){
        Pade(num,ordenNum,den,ordenDen);
    }else{
        ponerCeros(num,ordenNum,den,ordenDen);
        double h[cantidad]={0.0};
        respuestaImpulso(cantidad,h);
        Prony(h,num, ordenNum, den, ordenDen,cantidad,false);
        Shank(h, num, ordenNum, den, ordenDen,cantidad);
    }
}

void FractionalOrderFilter::ponerCeros(double* num,int ordenNum,double* den,int ordenDen){
        memset(den, 0, (ordenDen + 1) * sizeof(double));
        memset(num, 0, (ordenNum + 1) * sizeof(double));
}