#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iomanip>
#include <time.h>
#include <cstdint>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <errno.h>
//#include "affichage.h"

#define MPU6050_ADDR      0x68
#define ACCEL_VAL      0x00 //0x02 //Permet d'activer l'autotest (3 bits poids fort) et configurer la plage de mesure des G (2bits suivants) les trois bits poids faible non utilisé ==> 000 10 000 ==> 0x02 (pas de test, 8g de plage)
#define WIDTH 50	//Taille de la barre d'affichage
#define MAX_G 2.0
#define GYRO_CONFIG 0x1B
#define MULTIPLEXEUR_ADDR 0x70
#define PORTS_ACTIF       0b10001111
#define TEMP_OUT_H        0x41
#define PWR_MGMT_1        0x6B
#define ACTIVE_OFFSET     0
#define NBDATASETREAD     5
#define NB_ECHANTILLONS   2000 //Nombre d'echantillon pour le calcul des offsets
#define DATA_IN_FIFO      0xF8
#define NBCAPTEURS        5
#define ACTIVE_KALMAN     1
#define LOGFILE           1

/*
* Definition de la structure pour le filtre de Kalman
*/
typedef struct {
    float Q_angle;    // Bruit du processus (angle)
    float Q_bias;     // Bruit du processus (biais gyro)
    float R_measure;  // Bruit de mesure (accéléromètre)

    float angle;      // Angle estimé
    float bias;       // Biais du gyroscope estimé
    float rate;       // Taux non biaisé

    float P[2][2];    // Matrice de covariance de l'erreur
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf) {
/*
kf->Q_angle   = 0.001f;
//  ↑ petit  → fait plus confiance au gyro (lissage)
//  ↑ grand  → suit mieux les changements rapides

kf->Q_bias    = 0.003f;
//  ↑ petit  → biais gyro supposé stable
//  ↑ grand  → biais gyro supposé variable

kf->R_measure = 0.03f;
//  ↑ petit  → fait confiance à l'accéléromètre
//  ↑ grand  → ignore l'accéléromètre (gyro dominant)
*/

    kf->Q_angle   = 0.001f;  // Bruit processus angle
    kf->Q_bias    = 0.003f;  // Bruit processus biais
    kf->R_measure = 0.03f;   // Bruit mesure accéléromètre

    kf->angle = 0.0f;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;

    // Initialisation de la matrice de covariance
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

//  ┌─────────────────────────────────────────────────────────────────┐
//  │  angle_mesure : angle calculé par l'accéléromètre (°)           │
//  │  gyro_rate    : vitesse angulaire du gyroscope (°/s)            │
//  │  dt           : temps écoulé depuis la dernière itération (s)   │
//  │  retourne     : angle filtré (°)                                │
//  └─────────────────────────────────────────────────────────────────┘
float Kalman_Update(KalmanFilter *kf, float newAngle, float newRate, float dt) {

    // =====================
    // ÉTAPE 1 : PRÉDICTION
    // =====================

    // Taux gyro sans biais
    kf->rate = newRate - kf->bias;

    // Prédiction de l'angle
    kf->angle += dt * kf->rate;

    // Mise à jour de la matrice de covariance
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // =====================
    // ÉTAPE 2 : CORRECTION
    // =====================

    // Innovation (erreur entre mesure et prédiction)
    float S = kf->P[0][0] + kf->R_measure;

    // Gain de Kalman
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Mise à jour de l'état
    float y = newAngle - kf->angle;  // Résidu
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // Mise à jour de la matrice de covariance
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* ═══════════════════════════════════════════════════════════════════════════
   Paramètres du FIFO entre les capteurs et les traitements
   ═══════════════════════════════════════════════════════════════════════════ */
#define FIFO_CPT_WOR       "/fifo_cpt_worker"
#define CAPACITY       10 //6          /* nb de slots dans le ring buffer  (capacité du FIFO) identique pour tous       */

/* ═══════════════════════════════════════════════════════════════════════════
   Paramètres du FIFO entre les traitements et conssomateur
   ═══════════════════════════════════════════════════════════════════════════ */
#define FIFO_WOR_CUS       "/fifo_worker_customer"


typedef struct {
    int8_t numcpt; //Numero du capteur (commence à 0)
    float ax, ay, az;  // accéléromètre (g)
    float gx, gy, gz;  // gyroscope (deg/s)
    float temperature;  //Lecture de la temperature

    /*Données brutes sorties de capteur*/
    int16_t brut_ax, brut_ay, brut_az; 
    int16_t brut_gx, brut_gy, brut_gz;
    int16_t brut_temperature;

    float dt;

} CAPTEUR_Data_set;

typedef struct {
    pthread_mutex_t mutex;
    float cumulgx[NBCAPTEURS]; //Cumul de la rotation sur les 3 axes
    float cumulgy[NBCAPTEURS]; //Cumul de la rotation sur les 3 axes
    float cumulgz[NBCAPTEURS]; //Cumul de la rotation sur les 3 axes
} CumulAngle;

CumulAngle *cumulangle = NULL;


/* ═══════════════════════════════════════════════════════════════════════════
   FIFO partagé (ring buffer + sémaphores)
   ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    CAPTEUR_Data_set slots[CAPACITY];
    int     head;         /* prochain index à lire   */
    int     tail;         /* prochain index à écrire */
    int     done;         /* émetteur terminé        */
    sem_t   sem_vide;     /* slots libres    (init = CAPACITY) */
    sem_t   sem_plein;    /* slots occupés   (init = 0)        */
    sem_t   sem_mutex;    /* section critique                  */
} FifoRAM;

/* ─── Empiler un message (NON BLOQUANT) ───────────────────────────────────
 * Retourne :
 *   1  → message enfilé avec succès
 *   0  → file pleine (réessayer plus tard)
 * ────────────────────────────────────────────────────────────────────────── */
static int fifo_push(FifoRAM *f, const CAPTEUR_Data_set *m)
{
    /* sem_trywait : retourne immédiatement si aucun slot libre */
    if (sem_trywait(&f->sem_vide) != 0)
        return 0;                   /* file pleine */

    sem_wait(&f->sem_mutex);
    f->slots[f->tail] = *m;
    f->tail = (f->tail + 1) % CAPACITY;
    sem_post(&f->sem_mutex);
    sem_post(&f->sem_plein);

    return 1;                       /* enfilé avec succès */
}

/* ─── Depiler un message (NON BLOQUANT) ───────────────────────────────────
 * Retourne :
 *   1  → message lu dans *m
 *   0  → file vide (réessayer plus tard)
 *  -1  → émetteur terminé ET file vide (arrêt définitif)
 * ────────────────────────────────────────────────────────────────────────── */
static int fifo_pop(FifoRAM *f, CAPTEUR_Data_set *m)
{
    /* sem_trywait : retourne immédiatement si aucun slot plein */
    if (sem_trywait(&f->sem_plein) != 0)
    {
        /* File vide — vérifie si l'émetteur a terminé */
        sem_wait(&f->sem_mutex);
        int fini = f->done && (f->head == f->tail);
        sem_post(&f->sem_mutex);
        return fini ? -1 : 0;   /* -1=terminé, 0=vide mais pas fini */
    }

    sem_wait(&f->sem_mutex);
    *m = f->slots[f->head];
    f->head = (f->head + 1) % CAPACITY;
    sem_post(&f->sem_mutex);
    sem_post(&f->sem_vide);

    return 1;   /* message lu avec succès */
}

typedef struct {
    float gyro_x_offset, gyro_y_offset, gyro_z_offset; //Offset pour corriger les capteurs
} CAPTEURS_Offset;

/* ═══════════════════════════════════════════════════════════════════════════
   Flag d'arrêt — volatile int + mutex
   volatile  : empêche le compilateur de mettre la valeur en cache registre
   mutex     : garantit la visibilité entre threads (mémoire cohérente)
   ═══════════════════════════════════════════════════════════════════════════ */
static volatile int    running = 1;
static pthread_mutex_t mtx_run = PTHREAD_MUTEX_INITIALIZER;

/* Flag dans mémoire partagée — visible par tous les processus fork() */
typedef struct {
    volatile int       running;
    pthread_mutex_t    mtx;
} SharedFlag;

SharedFlag *flag = NULL;

static void flag_stop(SharedFlag *f) {
    pthread_mutex_lock(&f->mtx);
    f->running = 0;
    pthread_mutex_unlock(&f->mtx);
}

static int flag_get(SharedFlag *f) {
    int v;
    pthread_mutex_lock(&f->mtx);
    v = f->running;
    pthread_mutex_unlock(&f->mtx);
    return v;
}

/* ═══════════════════════════════════════════════
   Thread d'écoute — attend le caractère 'q'
   ═══════════════════════════════════════════════ */
static void *keyboard_listener(void *arg)
{
    SharedFlag *flag = (SharedFlag *)arg;
    printf("  [ecoute ]  tapez  q + Entree  pour tout arreter\n");

    int c;
    while ((c = getchar()) != EOF) {
        if (c == 'q' || c == 'Q') {
            printf("\n  [ecoute clavier : ] ->  'q' recu, arret de toutes les taches...\n\n");
            fflush(stdout);
            flag_stop(flag);
            break;
        }
    }
    return NULL;
}

/*
* Permet d'ouvrir un flux vers un capteur
*/

int open_flux(char addr_capt){
    const char *device = "/dev/i2c-1";
    //Ouverture du flux vers le multiplexer et les capteurs
    int file;
    if ((file = open(device, O_RDWR)) < 0) {
        std::cerr << "Erreur ouverture I2C\n";
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, addr_capt) < 0) {
        std::cerr << "Erreur selection esclave\n";
        return 1;
    }
    return file;
}

/*
* Fonctions de lecture / ecriture avec controle d'erreur 
*/
bool _read(int file, char data_read[], char l){
	ssize_t bytesRead = read(file, data_read, l);
    if (bytesRead < l) {
    	std::cout << "erreur de read "<< std::endl;
//        exit(0);
          return 0;
     }
     return 1;
}
void _write(int file, char data_write[], char l){
    ssize_t bytesWrite = bytesWrite = write(file, data_write, l);
    if (bytesWrite < l) {
        std::cout << "erreur de write "<< std::endl;
        exit(0);
    }
}

/*
* Fonction pour parametrer les fifo de tous les capteurs et positionner une vitesse d'echantillonage elevée
*/
void initallcapteur(int file_multi, int file_capteur){
	//On active dans le multiplexer tous le routage vers tous les capteurs
	char portsactif[1] = {PORTS_ACTIF};
	_write(file_multi, portsactif, 1);

	char param[2] = {0}; //uint8_t ==> a voir
    // Wake up
    param[0] = 0x6B;
//    param[1] = 0x00;
    param[1] = 0x01;
    _write(file_capteur, param, 2);

    //Activation du filtre Gyro
    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    param[0] = 0x1A;
//	param[1] = 0x03; // ~42 Hz
	param[1] = 0x01; // ~188 Hz
	_write(file_capteur, param, 2);

    // Sample rate = 1kHz
    param[0] = 0x19;
//    param[1] = 0x07;
    param[1] = 0x00;
//    param[1] = 0x09; // 1kHz sample rate (0x00 = 8kHz, 0x07 = 1kHz) 0x9 = 100 (Fonctionne)
    _write(file_capteur, param, 2);
    
    //Configuration de la plage des gyro
    param[0] = GYRO_CONFIG;
    param[1] = 0x00;
    _write(file_capteur, param, 2);
    
    // Accel Valeur defini en parametre
    param[0] = 0x1C;
    param[1] = ACCEL_VAL;
    _write(file_capteur, param, 2);

    // Reset FIFO
    param[0] = 0x6A;
//    param[1] = 0x04;
    param[1] = 0x07;
    _write(file_capteur, param, 2);

    // On met les capteurs temperature accelerateurs & les gyro dans le FIFO
    param[0] = 0x23;
    param[1] = DATA_IN_FIFO;
    _write(file_capteur, param, 2);

   // Enable FIFO
    param[0] = 0x6A;
//    param[1] = 0x44;
    param[1] = 0x47;
//    param[1] = 0x67;
    _write(file_capteur, param, 2);
	
	//On re initialise le multiplxeur (pour fermer tous les ports)
	portsactif[1] = 0x00;
	_write(file_multi, portsactif, 1);
}

/*
Simple :  ┌ ┐ └ ┘ ─ │ ├ ┤ ┬ ┴ ┼
Double :  ╔ ╗ ╚ ╝ ═ ║ ╠ ╣ ╦ ╩ ╬
Épais  :  ┏ ┓ ┗ ┛ ━ ┃ ┣ ┫ ┳ ┻ ╋
*/

void affichage(CAPTEUR_Data_set data_cpt, bool firstdisplay, float *cumultime){
    //printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    std::ostringstream buffer;
    int delta_aff = 10; //Nombre de ligne de l'affichage (pour se positionner au bon endroit)//On laisse les ligne avec les messages de start
    //Afficher la premiere ligne
    if(firstdisplay){
        //On efface toute la console
        buffer << "\033[" << 1 << ";" << 1 << "H";
        //Delata affichage
        printf("\033[%d;%dH", delta_aff, 0);
        printf(   "┏━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━━━┓\n"); //65 caracteres de large
        printf(   "┃                ┃  Capteur 0   ┃  Capteur 1   ┃  Capteur 2   ┃  Capteur 3   ┃  Capteur 4  ┃\n");
        printf(   "┣━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━┻━━━━━━━━━━━━━┫\n");
        printf(   "┃ Acceleration X │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Acceleration Y │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Acceleration Z │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Temperature    │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Gyro (brut)  X │              │              │              │              │             ┃\n");
        printf(   "┃ Gyro (corrige) │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Gyro (brut)  Y │              │              │              │              │             ┃\n");
        printf(   "┃ Gyro (corrige) │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ Gyro         Z │              │              │              │              │             ┃\n");
        printf(   "┣────────────────┼──────────────┼──────────────┼──────────────┼──────────────┼─────────────┫\n");
        printf(   "┃ dt.            │              │              │              │              │             ┃\n");
        printf(   "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
        if(LOGFILE){
            FILE *f = fopen("gyro_log.csv", "w");
            if (f) {
                fprintf(f, "Time;RawX;AngleX_Kalman;RawY;AngleY_Kalman;RawZ;\n");
                fclose(f);
            }
        }
    }
 
    //Position Y
        //accel_X ==> 4
        //accel_Y ==> 6
        //accel_Z ==> 8
        //temperature ==> 10
        //gyro_X ==> 12
        //gyro_Y ==> 14
        //gyro_Z ==> 16

    //Position X
        //capteur 0 ==> 18
        //capteur 1 ==> 32
        //capteur 2 ==> 46
        //capteur 3 ==> 60
        //capteur 4 ==> 74

    int y = 0;
    int x =0;

    switch (data_cpt.numcpt) {
        case 0:
            x = 20;
            break;
        case 1:
            x = 35;
            break;
        case 2:
            x = 50;
            break;
        case 3:
            x = 65;
            break;
        case 4:
            x = 80;
            break;
        default:
            x = -1; // Valeur par défaut en cas de numéro de capteur invalide//Ne doit jamais arriver !
            break;
    }

    /*
    pthread_mutex_lock(&cumulangle->mutex);
    float cux = cumulangle->cumulgx[data_cpt.numcpt];
    float cuy = cumulangle->cumulgy[data_cpt.numcpt];
    float cuz = cumulangle->cumulgz[data_cpt.numcpt];
    pthread_mutex_unlock(&cumulangle->mutex);
    */
    float cux = data_cpt.brut_gx;
    float cuy = data_cpt.brut_gy;
    float cuz = data_cpt.brut_gz;

    delta_aff--;
    printf("\033[?25l");   // cacher le curseur
    y = 4+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.ax);
    y = 6+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.ay);
    y = 8+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.az);
    y = 10+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.temperature);

    y = 12+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", cux);
    y = 13+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.gx);

    y = 15+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne Hq

    printf("%9.5f", cuy);
    y = 16+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.gy);

    y = 18+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", data_cpt.gz);
    y = 20+delta_aff;
    printf("\033[%d;%dH", y, x);  // ESC [ ligne ; colonne H
    printf("%9.5f", 1/data_cpt.dt);

    printf("\033[%d;%dH", delta_aff+20, 0);  // Positionne le curseur apres le tableau
    printf("\033[?25h");   // réafficher le curseur
    *cumultime += data_cpt.dt;
    if(LOGFILE){
            FILE *f = fopen("gyro_log.csv", "a");
            if (f) {
                fprintf(f, "%f;%f;%f;%f;%f;%f;\n", *cumultime, cux, data_cpt.gx, cuy, data_cpt.gy, cuz);
                fclose(f);
            }
        }
}

/*
* Permet de conssomer les données (apres leurs traitements)
*/
static void consumer(FifoRAM *fifo){
    printf(" DEBUG : [IN consumer pid=%d] démarré\n", getpid());
    CAPTEUR_Data_set data_cpt;
    bool firstdisplay = true;
    float cumultime = 0.0;

    while (flag->running) {
        if(fifo_pop(fifo, &data_cpt)){
            affichage(data_cpt, firstdisplay, &cumultime);
            if(firstdisplay) firstdisplay = false;
        }
        else{ //Pas de data, on temporise
                usleep(1); //On est plus temps reel
        }
    }
    exit(EXIT_SUCCESS);
}

/*
* Permet de faire les traitements sur les données apres lecture des capteurs
*/
static void traitements(FifoRAM *fifo, FifoRAM *fifocustomer){
    printf(" DEBUG : [IN traitements pid=%d] démarré\n", getpid());
    //Application des traitements sorties de capteurs (Datasheet)
    CAPTEUR_Data_set data_cpt;
    KalmanFilter kf_roll, kf_pitch;
    if(ACTIVE_KALMAN){
        /*
        * Initialisation des data pour filtre de Kalman (A passer dans le main ?)
        */
        Kalman_Init(&kf_roll);
        Kalman_Init(&kf_pitch);
    }
    //Tant qu'il y a des données dans le FIFO
    while (flag->running) {
        if(fifo_pop(fifo, &data_cpt)){
            /* ── Seuil de zéro : ignorer le bruit sous 0.1 °/s ── */
            /* @TODO à degager la division par zero pas belle !!!!*/
            if (data_cpt.brut_ax < 0.1f && data_cpt.brut_ax > -0.1f) data_cpt.gx = 0.0f;
            if (data_cpt.brut_gy < 0.1f && data_cpt.brut_gy > -0.1f) data_cpt.gy = 0.0f;
            if (data_cpt.brut_gz < 0.1f && data_cpt.brut_gz > -0.1f) data_cpt.gz = 0.0f;

            data_cpt.ax = (float)data_cpt.brut_ax / 16384.0f; //Convertir les données brutes en g
            data_cpt.ay = (float)data_cpt.brut_ay / 16384.0f; //Convertir les données brutes en g
            data_cpt.az = (float)data_cpt.brut_az / 16384.0f; //Convertir les données brutes en g

            data_cpt.gx = (float)data_cpt.brut_gx / 131.0f; //Convertir les données brutes en deg/s
            data_cpt.gy = (float)data_cpt.brut_gy / 131.0f; //Convertir les données brutes en deg/s
            data_cpt.gz = (float)data_cpt.brut_gz / 131.0f; //Convertir les données brutes en deg/s

            data_cpt.temperature = (float)(data_cpt.brut_temperature / 340.0f) + 36.53f; //Convertir les données brutes en °C

            data_cpt.brut_gx = data_cpt.gx;
            data_cpt.brut_gy = data_cpt.gy;
            data_cpt.brut_gz = data_cpt.gz;

            /*
            * Ne fonctionne pas bien, on a des valeurs de cumul qui partent en vrille (tres grand ou tres petit) et qui ne correspondent pas à la réalité
            */
           /*
            pthread_mutex_lock(&cumulangle->mutex);
            cumulangle->cumulgx[data_cpt.numcpt] += data_cpt.gx*data_cpt.dt;
            cumulangle->cumulgy[data_cpt.numcpt] += data_cpt.gy*data_cpt.dt;
            cumulangle->cumulgz[data_cpt.numcpt] += data_cpt.gz*data_cpt.dt;
            pthread_mutex_unlock(&cumulangle->mutex);
            */

            if(ACTIVE_KALMAN){
                float X = atan2f(data_cpt.ay, data_cpt.az) * (180.0f / M_PI);
        		float Y = -atan2f(data_cpt.ax, sqrtf(data_cpt.ay*data_cpt.ay + data_cpt.az*data_cpt.az)) * (180.0f / M_PI);
                data_cpt.gx = Kalman_Update(&kf_roll,  X,  data_cpt.gx, data_cpt.dt); //Rotation X Acceleration sur l'axe, rotation en °/S sur l'axe
                data_cpt.gy = Kalman_Update(&kf_pitch, Y, data_cpt.gy, data_cpt.dt);
            }

            //Integration des données dans le fifo
            if(!fifo_push(fifocustomer, &data_cpt)){
                //Si le fifo est plein, on attend un peu et on reessaye
                printf(" DEBUG : **** FIFO CUSTOMER PLEIN : PERTE DATA !!! ****\n\n");
                usleep(1); //Attendre 1ms avant de reessayer
            }

        }
        else{ //Pas de data, on temporise
            usleep(1); //On est plus temps reel
        }
    }
    exit(EXIT_SUCCESS);
}

void resetFIFO(int file)
{
    uint8_t data[2];
    data[0] = 0x6A;
    data[1] = 0x47; // FIFO_RESET + FIFO_EN
    write(file, data, 2);
}
uint16_t readFIFOCount(int file)
{
    uint8_t reg = 0x72;
    uint8_t data[2];

    write(file, &reg, 1);
    read(file, data, 2);

    return (data[0] << 8) | data[1];
}

/*
* Permet de mesuer le temps ecoulé depuis la derniere itération
*/
/*
// Pour un dt physique (gyroscope, intégration) → float en secondes
float dt = (float)(a->tv_sec  - b->tv_sec)
         + (float)(a->tv_nsec - b->tv_nsec) * 1e-9f;

// Pour mesurer des durées précises (profiling, logs) → int64_t en µs
int64_t dt_us = (int64_t)(a->tv_sec  - b->tv_sec) * 1000000LL
              + (int64_t)(a->tv_nsec - b->tv_nsec) / 1000LL;
*/
float timespec_diff_sec(struct timespec *a, struct timespec *b)
{
    return (float)(a->tv_sec  - b->tv_sec)
                 + (float)(a->tv_nsec - b->tv_nsec) * 1e-9f; 
}

int64_t timespec_diff_us(struct timespec *a, struct timespec *b)
{
    
    return (int64_t)(a->tv_sec  - b->tv_sec ) * 1000000LL
         + (int64_t)(a->tv_nsec - b->tv_nsec) / 1000LL;
    
    
    /*
    return (int64_t)(a->tv_sec  - b->tv_sec)
                 + (a->tv_nsec - b->tv_nsec) * 1e-9f; 
    */
    
}
void timespec_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

/*
* On va lire "samples" valeurs (parametre samples) pour chaque axe. On fait d'un coup (et donc on vide le FIFO)
*/
bool readBurstFIFO(int file, FifoRAM *fifo, int numcapt, struct timespec rt_cpt_clock[NBCAPTEURS])
{
    /*Reveil du capteur*/
    char config[2] = {PWR_MGMT_1, 0x01};
    _write(file, config, 2); 

    /* Vérifie un éventuel overflow (bit 4 de INT_STATUS) */
    uint8_t reg = 0x3A;
    char statusFIFO;
    write(file, &reg, 1);
    _read(file, &statusFIFO, 1);
    if (statusFIFO & 0x10) {
        resetFIFO(file);
        return false;   /* données corrompues → caller réessaiera */
    }
    if (statusFIFO & 0x00) {
        return false;   /* données pas pretes */
    }
    
    uint16_t count = readFIFOCount(file); //Combien de bits sont dans le FIFO

    //Le FIFO n'est pas suffisament plein (pas toutes les datas )
    if(count < 14)
        return false;

    reg = 0x74;
    char buffer[14] = {0};
    int16_t tmp = 0;
    write(file, &reg, 1);

    /*Lecture des valeurs du capteur*/
    CAPTEUR_Data_set data_cpt;
    data_cpt.numcpt = numcapt;

    //Lecture de l'horloge RT
    struct timespec rt_clock;
    clock_gettime(CLOCK_MONOTONIC, &rt_clock); //Recupere l'heure de l'horloge temps reel  
 
    //    float dt = (float) timespec_diff_us(&rt_clock, &rt_cpt_clock[numcapt]);
//	dt = dt/1000000.0;  //Convertir en seconde

    float dt = (rt_clock.tv_sec  - rt_cpt_clock[numcapt].tv_sec)
                 + (rt_clock.tv_nsec - rt_cpt_clock[numcapt].tv_nsec) * 1e-9f;

    rt_cpt_clock[numcapt] = rt_clock;

  //  dt = timespec_diff_sec(&rt_clock, &rt_cpt_clock[numcapt]);
    data_cpt.dt = dt;

    if(_read(file, buffer, 14)){
        data_cpt.brut_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
        data_cpt.brut_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
        data_cpt.brut_az = (int16_t)((buffer[4] << 8) | buffer[5]);
        data_cpt.brut_temperature = (int16_t)((buffer[6] << 8) | buffer[7]);
        data_cpt.brut_gx = (int16_t)((buffer[8] << 8) | buffer[9]);
        data_cpt.brut_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
        data_cpt.brut_gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    }

    //Integration des données dans le fifo
    if(!fifo_push(fifo, &data_cpt)){
        //Si le fifo est plein, on attend un peu et on reessaye
        printf(" DEBUG : **** FIFO PLEIN traitements trop longs STOP !!! ****\n\n");
        usleep(1); //Attendre 1ms avant de reessayer
    }

    return true;
}

/*
On va lire les données brutes
Les données brutes sont mises dans le FIFO
Un processus de traitement vient conssomer les donner brutes pour y appliquer les traitements (filtre de Kalman, calcul d'angle, etc)
Les données sont ensuite mise dans un autre FIFO pour etre conssomer (affichage, stockage, etc)
*/

int main() {
    /* Lance le thread d'écoute pour la saisie du caractere d'arret*/
    flag = (SharedFlag *)mmap(NULL, sizeof(SharedFlag), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    /* Initialisation du mutex en mode PARTAGÉ entre processus */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED); /* ← crucial */
    pthread_mutex_init(&flag->mtx, &attr);
    flag->running = 1;

    pthread_t tid_kbd;
    pthread_create(&tid_kbd, NULL, keyboard_listener, flag);

	//Ouverture des flux
	int file_multi = open_flux(MULTIPLEXEUR_ADDR);
    int file_capteur = open_flux(MPU6050_ADDR);

    //Initialisation du multiplexeur
    char initmultiplex[2] = {MULTIPLEXEUR_ADDR, 0x00};
    _write(file_multi, initmultiplex, 2);
    
    //Initialisation des capteurs pour utiliser leur FIFO, frequence et parametre plage accelerometre
    initallcapteur(file_multi, file_capteur);

    /* Créer le segment partagé CumulAngle *cumulangle = NULL;*/
    cumulangle = (CumulAngle *)mmap(NULL, sizeof(CumulAngle),
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANONYMOUS,
                        -1, 0);
    pthread_mutexattr_t attr2;
    pthread_mutexattr_init(&attr2);
    pthread_mutexattr_setpshared(&attr2, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&cumulangle->mutex, &attr2);
    pthread_mutexattr_destroy(&attr2); 
  
    /*
    * Initialisation des FIFO pour partager les données entre les taches 
    */

   /* ── 1. Créer le segment mémoire partagé dans /dev/shm (RAM pure) ── */
    shm_unlink(FIFO_CPT_WOR);   /* nettoie un éventuel résidu */
    shm_unlink(FIFO_WOR_CUS);   /* nettoie un éventuel résidu */

    int fd = shm_open(FIFO_CPT_WOR, O_CREAT | O_RDWR, 0666);
    int fd2 = shm_open(FIFO_WOR_CUS, O_CREAT | O_RDWR, 0666);
    if (fd == -1) { perror("shm_open"); return EXIT_FAILURE; }
    if (fd2 == -1) { perror("shm_open"); return EXIT_FAILURE; }

    if (ftruncate(fd, sizeof(FifoRAM)) == -1) {
        perror("ftruncate"); return EXIT_FAILURE;
    }
    if (ftruncate(fd2, sizeof(FifoRAM)) == -1) {
        perror("ftruncate"); return EXIT_FAILURE;
    }

    /* ── 2. Mapper en mémoire ── */
    FifoRAM *fifo = (FifoRAM *)mmap(
        NULL, sizeof(FifoRAM),
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    FifoRAM *fifocustomer = (FifoRAM *)mmap(
        NULL, sizeof(FifoRAM),
        PROT_READ | PROT_WRITE, MAP_SHARED, fd2, 0);
    close(fd);
    close(fd2);
    if (fifo == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }
    if (fifocustomer == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    /* ── 3. Initialiser le FIFO ── */
    memset(fifo, 0, sizeof(FifoRAM));
    memset(fifocustomer, 0, sizeof(FifoRAM));
    /* pshared=1 : sémaphores visibles entre processus distincts */
    sem_init(&fifo->sem_vide,  1, CAPACITY);
    sem_init(&fifo->sem_plein, 1, 0);
    sem_init(&fifo->sem_mutex, 1, 1);
    sem_init(&fifocustomer->sem_vide,  1, CAPACITY);
    sem_init(&fifocustomer->sem_plein, 1, 0);
    sem_init(&fifocustomer->sem_mutex, 1, 1);

    printf("  Segment RAM '%s' créé (%zu octets)\n",
           FIFO_CPT_WOR, sizeof(FifoRAM));
    printf("  Segment RAM '%s' créé (%zu octets)\n",
           FIFO_WOR_CUS, sizeof(FifoRAM));
 
           /*
    * Fin initialisation de la mémoire
    */

      /* ── 4. Fork : main lit les capteurs traitement traite ── */
    pid_t pid, pid_consumer;
//    pthread_atfork(before_fork, after_parent, after_child);
    pid = fork(); //Creatioin du processus de traitement des données (filtre de Kalman, calcul d'angle, etc)
     if (pid < 0) { perror("fork"); return EXIT_FAILURE; }

    if (pid == 0) {
        //Fonction de traitement des données (filtre de Kalman, calcul d'angle, etc)
        traitements(fifo, fifocustomer);      /* tache qui traite les données sorties de capteurs */
        exit(0);
    } 
    else{
        //Creation de la tache pour conssomer les datas
//        pthread_atfork(before_fork, after_parent, after_child);
        pid_consumer = fork();
        if (pid_consumer < 0) { perror("fork"); return EXIT_FAILURE;}
        if (pid_consumer == 0) {
            //Fonction de traitement des données (filtre de Kalman, calcul d'angle, etc)
            consumer(fifocustomer);      /* tache qui traite les données sorties de capteurs */
            exit(0);
        } 
        else{
            /*
            * Lecture des capteurs en boucle
            */
            struct timespec t;  //Boucle temps reel
            struct timespec rt_cpt_clock[NBCAPTEURS];
            float cumulanglecpt[NBCAPTEURS] = {0}; //Cumul de la rotation pour chaque capteur

            for(int t=0; t<NBCAPTEURS; t++){
                clock_gettime(CLOCK_MONOTONIC, &rt_cpt_clock[t]);
                pthread_mutex_lock(&cumulangle->mutex);
                cumulangle->cumulgx[t] = 0;
                cumulangle->cumulgy[t] = 0;
                cumulangle->cumulgz[t] = 0;
                pthread_mutex_unlock(&cumulangle->mutex);
            }

            while(flag->running){
                timespec_add_ns(&t, 2000000L); //Les capteurs crachent à 200Hz pas besoin d'aller plus que 500Hz pour les cycles de lecture
                char numcapteur = 0;
                for (char i = 0; i < 8; i++) { //8 est le nb max de capteur sur le multiplexeur
                    //Si port actif
                    if(PORTS_ACTIF & (1 << i)) {
                        //Activer le port sur le multiplexer
                        char t = 0;
                        t |= (1 << i);   // met le bit i à 1 inverssement c &= ~(1 << 2);  // met le bit 2 à 0
                        char addrcapteur[2] = {MULTIPLEXEUR_ADDR, t};
                        _write(file_multi, addrcapteur, 2);
                        //Attendre que le multiplexeur ai fait son taf
                        char etamultiplex[1] = {0};
                        while(1){
                            _read(file_multi, etamultiplex, 1);
                            if(etamultiplex[0] == t) break;
                            else std::cout << "attente multiplexeur "<< std::hex << (int)etamultiplex[0] << " " << t << std::endl;
                            usleep(1);
                        }
                        readBurstFIFO(file_capteur, fifo, numcapteur, rt_cpt_clock);
                        numcapteur ++;
                    }
                }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL); //Attente de xx depuis le debut de la boucle (temps reel) pour la prochaine lecture des capteurs
//                sleep(1);
            }
        }
    }
 

    waitpid(pid, NULL, 0); //On attend que la tache de traitement se termine
    waitpid(pid_consumer, NULL, 0); //On attend que la tache de traitement se termine

   /* ── 5. Nettoyage ── */
    sem_destroy(&fifo->sem_vide);
    sem_destroy(&fifo->sem_plein);
    sem_destroy(&fifo->sem_mutex);
    munmap(fifo, sizeof(FifoRAM));
    shm_unlink(FIFO_CPT_WOR);   /* supprime /dev/shm/fifo_shm */
}
