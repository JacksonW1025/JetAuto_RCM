//
// Created by Lucas on 21/7/2022.
//

#ifndef __ST7735_REG_H_
#define __ST7735_REG_H_

// commands
#define ST7735_NOP 0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID 0x04
#define ST7735_RDDST 0x09

#define ST7735_SLPIN		0x10
#define ST7735_SLPOUT		0x11
#define ST7735_PTLON		0x12
#define ST7735_NORON		0x13



#define ST7735_INVOFF		0x20
#define ST7735_INVON		0x21
#define ST7735_GAMSET		0x26      // Gamma set
#define ST7735_DISPOFF		0x28
#define ST7735_DISPON		0x29
#define ST7735_CASET		0x2A
#define ST7735_RASET		0x2B
#define ST7735_RAMWR		0x2C
#define ST7735_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7735_RAMRD		0x2E


#define ST7735_PTLAR   0x30
#define ST7735_VSCRDEF 0x33
#define ST7735_COLMOD 0x3A
#define ST7735_MADCTL 0x36
#define ST7735_VSCSAD 0x37

#define ST7735_MADCTL_MY 0x80
#define ST7735_MADCTL_MX 0x40
#define ST7735_MADCTL_MV 0x20
#define ST7735_MADCTL_ML 0x10
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04
#define ST7735_MADCTL_RGB 0x00

#define ST7735_RDID1 0xDA
#define ST7735_RDID2 0xDB
#define ST7735_RDID3 0xDC
#define ST7735_RDID4 0xDD


#define ST7735_RDDPM		0x0A      // Read display power mode
#define ST7735_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7735_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7735_RDDIM		0x0D      // Read display image mode
#define ST7735_RDDSM		0x0E      // Read display signal mode
#define ST7735_RDDSR		0x0F      // Read display self-diagnostic result (ST7735V)



#define ST7735_PTLAR		0x30
#define ST7735_VSCRDEF		0x33      // Vertical scrolling definition (ST7735V)
#define ST7735_TEOFF		0x34      // Tearing effect line off
#define ST7735_TEON			0x35      // Tearing effect line on
#define ST7735_MADCTL		0x36      // Memory data access control
#define ST7735_VSCRSADD		0x37      // Vertical screoll address
#define ST7735_IDMOFF		0x38      // Idle mode off
#define ST7735_IDMON		0x39      // Idle mode on
#define ST7735_RAMWRC		0x3C      // Memory write continue (ST7735V)
#define ST7735_RAMRDC		0x3E      // Memory read continue (ST7735V)
#define ST7735_COLMOD		0x3A

#define ST7735_RAMCTRL		0xB0      // RAM control
#define ST7735_RGBCTRL		0xB1      // RGB control
#define ST7735_PORCTRL		0xB2      // Porch control
#define ST7735_FRCTRL1		0xB3      // Frame rate control
#define ST7735_PARCTRL		0xB5      // Partial mode control
#define ST7735_GCTRL		0xB7      // Gate control
#define ST7735_GTADJ		0xB8      // Gate on timing adjustment
#define ST7735_DGMEN		0xBA      // Digital gamma enable
#define ST7735_VCOMS		0xBB      // VCOMS setting
#define ST7735_LCMCTRL		0xC0      // LCM control
#define ST7735_IDSET		0xC1      // ID setting
#define ST7735_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7735_VRHS			0xC3      // VRH set
#define ST7735_VDVSET		0xC4      // VDV setting
#define ST7735_VCMOFSET		0xC5      // VCOMS offset set
#define ST7735_FRCTRL2		0xC6      // FR Control 2
#define ST7735_CABCCTRL		0xC7      // CABC control
#define ST7735_REGSEL1		0xC8      // Register value section 1
#define ST7735_REGSEL2		0xCA      // Register value section 2
#define ST7735_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7735_PWCTRL1		0xD0      // Power control 1
#define ST7735_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7735_CMD2EN		0xDF      // Command 2 enable
#define ST7735_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7735_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7735_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7735_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7735_GATECTRL		0xE4      // Gate control
#define ST7735_SPI2EN		0xE7      // SPI2 enable
#define ST7735_PWCTRL2		0xE8      // Power control 2
#define ST7735_EQCTRL		0xE9      // Equalize time control
#define ST7735_PROMCTRL		0xEC      // Program control
#define ST7735_PROMEN		0xFA      // Program mode enable
#define ST7735_NVMSET		0xFC      // NVM setting
#define ST7735_PROMACT		0xFE      // Program action



#define COLOR_MODE_65K 0x50
#define COLOR_MODE_262K 0x60
#define COLOR_MODE_12BIT 0x03
#define COLOR_MODE_16BIT 0x05
#define COLOR_MODE_18BIT 0x06
#define COLOR_MODE_16M 0x07

#endif //RP_MPU_SRC_ST7735_REG_H_
