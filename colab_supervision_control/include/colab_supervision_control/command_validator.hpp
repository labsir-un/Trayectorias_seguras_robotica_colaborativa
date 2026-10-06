#ifndef COLAB_SUPERVISION_COMMAND_VALIDATOR__COMMAND_VALIDATOR_HPP_
#define COLAB_SUPERVISION_COMMAND_VALIDATOR__COMMAND_VALIDATOR_HPP_

#include <string>
#include <vector>
#include <sstream>

namespace colab_supervision_control
{

struct ValidationResult
{
  bool valid;
  std::string reason;
};

class CommandValidator
{
public:

  static ValidationResult validate(
    const std::string & command,
    const std::string & object,
    const std::string & variable,
    const std::string & value)
  {
    if (command.empty()) {
      return {false, "El campo 'command' esta vacio"};
    }

    if (command == "Velocidad") {
      return validateVelocidad(object, variable, value);
    }

    if (command == "Distancia") {
      return validateDistancia(object, variable, value);
    }

    if (command == "Variable") {
      return validateVariable(object, variable, value);
    }

    return {
      false,
      "Comando no reconocido: " + command
    };
  }

private:

  // ============================================================
  // VELOCIDAD
  // ============================================================

  static ValidationResult validateVelocidad(
    const std::string & object,
    const std::string & variable,
    const std::string & value)
  {
    // RAPID:
    //
    // CASE "Porcentaje"
    //   CASE "VelMax"
    //
    // CASE "Max"
    //   CASE "VelMax"
    //
    // CASE "ACTVEL"
    //   variable/value dinamicos

    if (object == "Porcentaje") {

      if (variable == "VelMax") {
        if (!value.empty()) {
          return {true, "Comando Velocidad/Porcentaje/VelMax valido"};
        }

        return {
          false,
          "Velocidad/Porcentaje/VelMax requiere un valor"
        };
      }

      return {
        false,
        "Variable no valida para Velocidad/Porcentaje: " + variable
      };
    }

    if (object == "Max") {

      if (variable == "VelMax") {
        if (!value.empty()) {
          return {true, "Comando Velocidad/Max/VelMax valido"};
        }

        return {
          false,
          "Velocidad/Max/VelMax requiere un valor"
        };
      }

      return {
        false,
        "Variable no valida para Velocidad/Max: " + variable
      };
    }

    if (object == "ACTVEL") {

      if (!variable.empty() && !value.empty()) {
        return {true, "Comando Velocidad/ACTVEL valido"};
      }

      return {
        false,
        "Velocidad/ACTVEL requiere variable y valor"
      };
    }

    return {
      false,
      "Objeto no valido para Velocidad: " + object
    };
  }


  // ============================================================
  // DISTANCIA
  // ============================================================

  static ValidationResult validateDistancia(
    const std::string & object,
    const std::string & variable,
    const std::string & value)
  {
    if (object == "GetPostura") {

      if (variable != "Prototipo") {
        return {
          false,
          "GetPostura requiere la variable 'Prototipo'"
        };
      }

      if (value == "TCPEuler" || value == "TCPCua") {
        return {
          true,
          "Comando Distancia/GetPostura valido"
        };
      }

      return {
        false,
        "Valor no valido para Prototipo: " + value
      };
    }


    if (object == "CambiarTrayectoria") {

      return validateCambiarTrayectoria(variable, value);
    }


    if (object == "Postura_segura") {

      if (variable == "robtargetJ" ||
          variable == "robtargetL" ||
          variable == "JoinConf")
      {
        if (!value.empty()) {
          return {
            true,
            "Comando Distancia/Postura_segura valido"
          };
        }

        return {
          false,
          "Postura_segura requiere un valor"
        };
      }

      return {
        false,
        "Variable no valida para Postura_segura: " + variable
      };
    }

    return {
      false,
      "Objeto no valido para Distancia: " + object
    };
  }


  // ============================================================
  // CAMBIAR TRAYECTORIA
  // ============================================================

  static ValidationResult validateCambiarTrayectoria(
    const std::string & variable,
    const std::string & value)
  {
    if (variable == "ActualizarPuntos") {

      if (value == "Interes" ||
          value == "GuardarPSalida")
      {
        return {
          true,
          "CambiarTrayectoria/ActualizarPuntos valido"
        };
      }

      return {
        false,
        "Valor no valido para ActualizarPuntos: " + value
      };
    }


    if (variable == "SetOffsets") {

      if (!value.empty()) {
        return {
          true,
          "CambiarTrayectoria/SetOffsets valido"
        };
      }

      return {
        false,
        "SetOffsets requiere un valor"
      };
    }


    if (variable == "MoveTemp") {

      if (value == "MoveJoint" ||
          value == "MoveLinear")
      {
        return {
          true,
          "CambiarTrayectoria/MoveTemp valido"
        };
      }

      return {
        false,
        "Valor no valido para MoveTemp: " + value
      };
    }


    return {
      false,
      "Variable no valida para CambiarTrayectoria: " + variable
    };
  }


  // ============================================================
  // VARIABLE
  // ============================================================

  static ValidationResult validateVariable(
    const std::string & object,
    const std::string & variable,
    const std::string & value)
  {
    if (object == "SET") {

      if (!variable.empty() && !value.empty()) {
        return {
          true,
          "Comando Variable/SET valido"
        };
      }

      return {
        false,
        "Variable/SET requiere variable y valor"
      };
    }


    if (object == "GET") {

      if (variable.empty()) {
        return {
          false,
          "Variable/GET requiere una variable"
        };
      }

      if (isValidGetType(value)) {
        return {
          true,
          "Comando Variable/GET valido"
        };
      }

      return {
        false,
        "Tipo no valido para Variable/GET: " + value
      };
    }


    return {
      false,
      "Objeto no valido para Variable: " + object
    };
  }


  // ============================================================
  // TIPOS GET
  // ============================================================

  static bool isValidGetType(
    const std::string & value)
  {
    return
      value == "num" ||
      value == "bool" ||
      value == "string" ||
      value == "speeddata" ||
      value == "zonedata" ||
      value == "wobjdataEuler" ||
      value == "wobjdataCua" ||
      value == "robtargetEuler" ||
      value == "robtargetCua" || 
      value == "joints";
  }
};

}    // namespace colab_supervision_control

#endif  // COLAB_SUPERVISION_COMMAND_VALIDATOR__COMMAND_VALIDATOR_HPP_