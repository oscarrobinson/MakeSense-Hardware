#include <stdio.h>
#include <sys/time.h>

double current_timestamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	double timestamp;
	double timestampMicro;
	double timestampSecond;
	timestampMicro = (double)tv.tv_usec; // microseconds
	timestampMicro = timestampMicro/1000000;
	timestampSecond = (double)tv.tv_sec;
	printf("us: %f\n", timestampMicro);
	timestamp = timestampMicro + timestampSecond;
	return timestamp;
}

int main(){
	double cTime;
	cTime = current_timestamp();
	printf ("%f\n",cTime);
	return 0;
}