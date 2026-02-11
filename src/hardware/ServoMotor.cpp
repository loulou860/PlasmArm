#include "ServoMotor.h"

<<<<<<< Updated upstream
ServoMotor::ServoMotor(uint8_t pwmPin)
    : pwmPin(pwmPin), currentAngle(0.0f), targetAngle(0.0f),
      speed(90.0f), enabled(false), isMovingFlag(false),
      lastUpdateTime(0), lastAngle(0.0f) {
=======
// namespace requis pour les ControlTableItem (OP_POSITION, PROFILE_VELOCITY, etc.)
using namespace ControlTableItem;

ServoMotor::ServoMotor(Dynamixel2Arduino& dxl, uint8_t dxl_id)
    : dxl(dxl),
      id(dxl_id),
      currentAngle(0.0f),
      targetAngle(0.0f),
      speed(90.0f),
      enabled(false),
      isMovingFlag(false),
      lastUpdateTime(0)
{
>>>>>>> Stashed changes
}

void ServoMotor::init(uint32_t baudrate, float protocol) {
    // Démarre le port Dynamixel
    dxl.begin(baudrate);

    // Définit la version du protocole
    dxl.setPortProtocolVersion(protocol);

    // Ping (si ça échoue: moteur pas joignable)
    if (!dxl.ping(id)) {
        // On ne fait pas Serial.print ici pour garder la classe générique.
        // Le main.cpp peut lire dxl.getLastLibErrCode() si besoin.
        enabled = false;
        isMovingFlag = false;
        return;
    }

    // Configuration en mode position
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);

    // Optionnel: limiter la vitesse “interne” Dynamixel (profil).
    // Ici on met une valeur modérée; ta rampe logicielle contrôle déjà la vitesse perçue.
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 30);

    dxl.torqueOn(id);
    enabled = true;

    // Synchroniser l'état logiciel sur la position réelle
    currentAngle = readPresentAngleDeg();
    targetAngle = currentAngle;
    isMovingFlag = false;
    lastUpdateTime = millis();
}

void ServoMotor::setSpeed(float speed_deg_s) {
    if (speed_deg_s < 1.0f) speed_deg_s = 1.0f;
    speed = speed_deg_s;
}

void ServoMotor::moveToAngle(float angle_deg) {
    // Normalise 0..360 comme avant
    while (angle_deg < 0) angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;

    // Clamp 0..180 comme avant
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    if (angle_deg < 0.0f)   angle_deg = 0.0f;

    targetAngle = angle_deg;

    // Rafraîchir currentAngle depuis le moteur (utile si on l'a bougé à la main / reboot)
    currentAngle = readPresentAngleDeg();

    isMovingFlag = (fabs(targetAngle - currentAngle) > 0.5f);
    lastUpdateTime = millis();
}

float ServoMotor::getCurrentAngle() {
    // On peut renvoyer l'état logiciel; ou lire direct le moteur.
    // Pour être fidèle à ton code (rampe): on renvoie currentAngle.
    return currentAngle;
}

void ServoMotor::enable() {
    dxl.torqueOn(id);
    enabled = true;

    // Si on est déjà à la cible, on envoie une consigne “stable”
    if (!isMovingFlag) {
        dxl.setGoalPosition(id, currentAngle, UNIT_DEGREE);
    }
}

void ServoMotor::disable() {
    dxl.torqueOff(id);
    enabled = false;
    isMovingFlag = false;
}

bool ServoMotor::isEnabled() {
    return enabled;
}

bool ServoMotor::isMoving() {
    return isMovingFlag && enabled;
}

void ServoMotor::stop() {
    // Stop = cible = position actuelle (lue du moteur)
    currentAngle = readPresentAngleDeg();
    targetAngle = currentAngle;
    isMovingFlag = false;
    dxl.setGoalPosition(id, currentAngle, UNIT_DEGREE);
}

float ServoMotor::readPresentAngleDeg() {
    float a = dxl.getPresentPosition(id, UNIT_DEGREE);

    // Selon mode / modèle, parfois ça peut dépasser, on clamp pour rester cohérent avec ton API 0..180
    if (a > 180.0f) a = 180.0f;
    if (a < 0.0f)   a = 0.0f;

    return a;
}

void ServoMotor::update() {
    if (!enabled || !isMovingFlag) {
        return;
    }

    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f; // secondes

    if (dt <= 0.0f) return;

    // Rampe logicielle identique à ton approche: on avance de speed*dt vers la cible
    float maxStep = speed * dt;
    float diff = targetAngle - currentAngle;

    if (fabs(diff) <= maxStep) {
        currentAngle = targetAngle;
        isMovingFlag = false;
    } else {
        currentAngle += (diff > 0.0f) ? maxStep : -maxStep;
    }

    // Clamp 0..180
    if (currentAngle > 180.0f) currentAngle = 180.0f;
    if (currentAngle < 0.0f)   currentAngle = 0.0f;

    // Envoie la nouvelle consigne au Dynamixel
    dxl.setGoalPosition(id, currentAngle, UNIT_DEGREE);

    lastUpdateTime = now;
}
