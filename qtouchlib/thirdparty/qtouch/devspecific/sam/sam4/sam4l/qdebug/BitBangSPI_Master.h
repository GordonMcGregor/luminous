#ifndef BITBANGSPI_MASTER_H_INCLUDED
#define BITBANGSPI_MASTER_H_INCLUDED

/*============================ PROTOTYPES ====================================*/
void BitBangSPI_Master_Init (void);
void BitBangSPI_Send_Message(void);

/*============================ MACROS ========================================*/
#define JOIN( x, y ) x ## y
#define REG( REGISTER, SIDE ) JOIN( REGISTER, SIDE )

#endif

/* EOF */
