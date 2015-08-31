#ifndef __IO_KEY_H__
#define __IO_KEY_H__

struct key_pad_plat_data {
    int  nbuttons;
    unsigned int *buttons;
    unsigned int *keycodes;
    unsigned int *active_high;
    int autorepeat;       /* key repeat 1 = on , 0 = off */
    int	resume_delay_ms;
};
#endif
