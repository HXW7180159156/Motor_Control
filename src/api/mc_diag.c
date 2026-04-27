/**
 * @file mc_diag.c
 * @brief Diagnostic utilities implementation
 */
#include "mc_diag.h"

/**
 * @brief Get string name for a 1-shunt compensation mode
 * @param mode Compensation mode enum
 * @return Constant string describing the mode ("none", "predict_basic", etc.)
 */
const char *mc_diag_1shunt_comp_mode_name(mc_1shunt_comp_mode_t mode)
{
    switch (mode)
    {
        case MC_1SHUNT_COMP_NONE:
            return "none";

        case MC_1SHUNT_COMP_PREDICT_BASIC:
            return "predict_basic";

        case MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION:
            return "predict_high_modulation";

        case MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING:
            return "predict_field_weakening";

        default:
            return "unknown";
    }
}

/**
 * @brief Translation unit anchor to ensure mc_diag symbols are linked
 * @return Value of MC_WARNING_NONE as int
 */
int mc_diag_translation_unit_anchor(void)
{
    return (int)MC_WARNING_NONE;
}
