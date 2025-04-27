#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <net/if.h>       
#include <sys/ioctl.h>    
#include <sys/socket.h>   

#include <linux/can.h> 
#include <linux/can/raw.h>

#define EXPECTED_CAN_ID 0x123
#define EXPECTED_DLC 6

int main(int argc, char **argv) {
    int s; // socket file descriptor
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int nbytes;            

    const char *ifname = "can1";

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket PF_CAN");
        return 1;
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(s);
        return 1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return 1;
    }

    printf("CAN if '%s' listening (ID: 0x%X)...\n", ifname, EXPECTED_CAN_ID);

    while (1) {
        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("read can raw");
            sleep(1);
            continue;
        }

        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Error read: less than expected byte (%d byte)\n", nbytes);
            continue;
        }

        if (!(frame.can_id & CAN_EFF_FLAG) && ((frame.can_id & CAN_SFF_MASK) == EXPECTED_CAN_ID))
        {
            if (frame.can_dlc == EXPECTED_DLC)
            {
                int16_t received_x = (int16_t)((frame.data[1] << 8) | frame.data[0]);
                int16_t received_y = (int16_t)((frame.data[3] << 8) | frame.data[2]);
                int16_t received_z = (int16_t)((frame.data[5] << 8) | frame.data[4]);

                printf("ID: 0x%03X DLC: %d | X: %d, Y: %d, Z: %d\n",
                       frame.can_id & CAN_SFF_MASK,
                       frame.can_dlc,
                       received_x,
                       received_y,
                       received_z);

                fflush(stdout);
            }
            else
            {
                fprintf(stderr, "ID 0x%X received but DLC wrong (expected: %d, received: %d)\n",
                        EXPECTED_CAN_ID, EXPECTED_DLC, frame.can_dlc);
            }
        }
    }

    close(s);
    return 0;
}
