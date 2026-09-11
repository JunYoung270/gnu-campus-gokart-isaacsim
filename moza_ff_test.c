#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#define DEVICE "/dev/input/moza-event"

int main(void)
{
    int fd = -1;
    int effects = 0;
    struct ff_effect effect;
    struct input_event play;
    struct input_event stop;

    printf("MOZA R5 low-force FFB test\n");
    printf("Device: %s\n", DEVICE);

    fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    if (ioctl(fd, EVIOCGEFFECTS, &effects) == 0) {
        printf("Supported simultaneous effects: %d\n", effects);
    } else {
        perror("EVIOCGEFFECTS");
    }

    memset(&effect, 0, sizeof(effect));

    effect.type = FF_CONSTANT;
    effect.id = -1;

    /*
     * Linux FF direction:
     * 0x4000 = steering X-axis direction
     *
     * Constant level range:
     * -32767 ~ +32767
     *
     * 327 ~= about 1%
     */
    effect.direction = 0x4000;
    effect.u.constant.level = -327;

    effect.trigger.button = 0;
    effect.trigger.interval = 0;

    effect.replay.length = 500;  /* 0.5 sec */
    effect.replay.delay = 0;

    if (ioctl(fd, EVIOCSFF, &effect) < 0) {
        perror("EVIOCSFF");
        close(fd);
        return 1;
    }

    printf("Effect uploaded. id=%d\n", effect.id);
    printf("Starting ~1%% constant force for 0.5 sec...\n");

    memset(&play, 0, sizeof(play));
    play.type = EV_FF;
    play.code = effect.id;
    play.value = 1;

    if (write(fd, &play, sizeof(play)) != sizeof(play)) {
        perror("play write");
        ioctl(fd, EVIOCRMFF, effect.id);
        close(fd);
        return 1;
    }

    usleep(500000);

    memset(&stop, 0, sizeof(stop));
    stop.type = EV_FF;
    stop.code = effect.id;
    stop.value = 0;

    if (write(fd, &stop, sizeof(stop)) != sizeof(stop)) {
        perror("stop write");
    }

    ioctl(fd, EVIOCRMFF, effect.id);
    close(fd);

    printf("FFB stopped and effect removed.\n");

    return 0;
}
