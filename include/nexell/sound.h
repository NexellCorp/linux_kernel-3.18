
/*
 *    Sound platform data
 */
#include <linux/dmaengine.h>
#include <sound/pcm.h>

/* I2S */
struct nxp_i2s_plat_data {
    int     master_mode;
    int     master_clock_in;
    int     trans_mode;                 /* 0:I2S, 1:Left 2:Right justified */
    int     sample_rate;
    int     sample_bit;                 /* support only 8, 16, 24 */
    int     frame_bit;                  /* support only 32, 48 */
    int     LR_pol_inv;
    int     pre_supply_mclk;            /* codec require mclk out, before codec initialize */
	bool	(*ext_is_en)(void);
	unsigned long (*set_ext_mclk)(unsigned long clk, int ch);
    bool    (*dma_filter)(struct dma_chan *chan, void *filter_param);
    const char *dma_play_ch;
    const char *dma_capt_ch;
};

/* SPDIF */
struct nxp_spdif_plat_data {
    int sample_rate;
    int hdmi_out;
    bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
    const char *dma_ch;
};

/* PDM */
struct nxp_pdm_plat_data {
    int sample_rate;
    bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
    const char *dma_ch;
};

/* sound DAI (I2S/SPDIF and codec interface) */
struct nxp_snd_jack_pin {
    int    support;
    int    detect_level;
    int detect_io;
    int debounce_time;
};

struct nxp_snd_dai_plat_data {
    int i2s_ch;
    unsigned int sample_rate;
    unsigned int pcm_format;        /* SNDRV_PCM_FMTBIT_S16_LE, SNDRV_PCM_FMTBIT_S24_LE, .. (include/sound/pcm.h) */
    struct nxp_snd_jack_pin hp_jack;
    struct nxp_snd_jack_pin mic_jack;
};
