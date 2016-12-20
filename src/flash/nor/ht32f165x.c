
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "../common.h"
#include "../../helper/command.h"

#include "imp.h"
#include <target/armv7m.h>

#define FMC_REG_BASE        0x40080000
#define FMC_REG_TADR        0x00
#define FMC_REG_WRDR        0x04
#define FMC_REG_OCMR        0x0C
#define FMC_REG_OPCR        0x10

#define FMC_CMD_WORD_PROG   0x4
#define FMC_CMD_PAGE_ERASE  0x8
#define FMC_CMD_MASS_ERASE  0xA

#define FMC_COMMIT          (0xA << 1)
#define FMC_FINISHED        (0xE << 1)

#define FLASH_ERASE_TIMEOUT 1000

// flash bank ht32f165x <base> <size> 0 0 <target#>
FLASH_BANK_COMMAND_HANDLER(ht32f165x_flash_bank_command)
{
    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;

    bank->driver_priv = NULL;

    return ERROR_OK;
}

static int ht32f165x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
    struct target *target = bank->target;
    return target_read_u32(target, FMC_REG_BASE + FMC_REG_OCMR, status);
}

static int ht32f165x_wait_status_busy(struct flash_bank *bank, int timeout)
{
    uint32_t status;
    int retval = ERROR_OK;

    /* wait for busy to clear */
    for (;;) {
        retval = ht32f165x_get_flash_status(bank, &status);
        if (retval != ERROR_OK)
            return retval;
//        LOG_DEBUG("status: 0x%" PRIx32 "", status);
        if ((status & FMC_FINISHED) == FMC_FINISHED)
            return ERROR_OK;
        if (timeout-- <= 0) {
            LOG_ERROR("timed out waiting for flash");
            return ERROR_FAIL;
        }
        alive_sleep(1);
    }

    return retval;
}

static int ht32f165x_erase(struct flash_bank *bank, int first, int last){
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int ht32f165x_protect(struct flash_bank *bank, int set, int first, int last){
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int ht32f165x_write(struct flash_bank *bank, const uint8_t *buffer,
                           uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if(offset & 0x3){
        LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

    if(count & 0x3){
        LOG_ERROR("size 0x%" PRIx32 " breaks required 4-byte alignment", count);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

    uint32_t *words = (uint32_t *)buffer;
    uint32_t addr = offset;
    for(uint32_t i = 0; i < count; i += 4){
        /* write flash memory */
        int retval;
        retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_TADR, addr);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_WRDR, *words);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_OCMR, FMC_CMD_WORD_PROG);
        if (retval != ERROR_OK)
            return retval;
        retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_OPCR, FMC_COMMIT);
        if (retval != ERROR_OK)
            return retval;

        retval = ht32f165x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
        if (retval != ERROR_OK)
            return retval;
        words++;
        addr += 4;
    }

    return ERROR_OK;
}

static int ht32f165x_probe(struct flash_bank *bank){
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int ht32f165x_auto_probe(struct flash_bank *bank){
    return ERROR_OK;
}

static int ht32f165x_protect_check(struct flash_bank *bank){
    return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int ht32f165x_info(struct flash_bank *bank, char *buf, int buf_size){
    const char *info = "ht32f165x flash";
    strncpy(buf, info, MIN((int)strlen(info), buf_size));
    return ERROR_OK;
}

static int ht32f165x_mass_erase(struct flash_bank *bank)
{
    struct target *target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    /* mass erase flash memory */
    int retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_OCMR, FMC_CMD_MASS_ERASE);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, FMC_REG_BASE + FMC_REG_OPCR, FMC_COMMIT);
    if (retval != ERROR_OK)
        return retval;

    retval = ht32f165x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

COMMAND_HANDLER(ht32f165x_handle_mass_erase_command)
{
    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    retval = ht32f165x_mass_erase(bank);
    if (retval == ERROR_OK) {
        /* set all sectors as erased */
        int i;
        for (i = 0; i < bank->num_sectors; i++)
            bank->sectors[i].is_erased = 1;

        command_print(CMD_CTX, "ht32f165x mass erase complete");
    } else {
        command_print(CMD_CTX, "ht32f16x mass erase failed");
    }

    return retval;
}


static const struct command_registration ht32f165x_exec_command_handlers[] = {
    {
        .name = "mass_erase",
        .handler = ht32f165x_handle_mass_erase_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Erase entire flash device.",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration ht32f165x_command_handlers[] = {
    {
        .name = "ht32f165x",
        .mode = COMMAND_ANY,
        .help = "ht32f165x flash command group",
        .usage = "",
        .chain = ht32f165x_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

struct flash_driver ht32f165x_flash = {
    .name = "ht32f165x",
    .commands = ht32f165x_command_handlers,
    .flash_bank_command = ht32f165x_flash_bank_command,

    .erase          = ht32f165x_erase,
    .protect        = ht32f165x_protect,
    .write          = ht32f165x_write,
    .read           = default_flash_read,
    .probe          = ht32f165x_probe,
    .auto_probe     = ht32f165x_auto_probe,
    .erase_check    = default_flash_blank_check,
    .protect_check  = ht32f165x_protect_check,
    .info           = ht32f165x_info,
};
