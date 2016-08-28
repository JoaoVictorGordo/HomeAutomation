#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define MaxPayload 32
#define Ncommands 2 // number of commands available

// --------- deadlines (in millisseconds)----------------------------
#define dt 1 // time increment for the deadlock handling routine
#define t_min 1
#define t_max 20 
#define t_com 50 // deadline for performing communication (snd & rcv)
#define n_max 10 // number of attemptives to handle deadlock

RF24 radio(5,6);

// Network uses that radio
RF24Network network(radio);

// Address of our node
const uint16_t this_node = 2;
// Address of the other node
uint16_t other_node = 0;
uint16_t write_node = 0;

char rcv[MaxPayload];
char cmd[MaxPayload];
char msg[MaxPayload];
bool reading; // reading/writing flag
unsigned long t;
bool wrt = 0;
bool to_whom = 0;

char **intCmd; // internal commands

// -------- communication oriented functions -------
void communication(bool mode);
bool Read_nonBlocking();
bool Read_blocking();
bool Write_nonBlocking(bool, unsigned uint16_t);
bool Write_blocking(bool, unsigned uint16_t);
void deadlockHandling(unsigned int);
void copyString(char *, char *);
void setInternalCommands(char**); // robot commands
void write_command(char *);
bool isSubstring(char *, char *);
bool processing();
void action(int );
int mypow(int, int);
int char2int(char *);
void write_command(char *);
void concatenate(char *, char *);
void int2char(char *,int );

void setup(void)
{
    Serial.begin(57600);

    SPI.begin();
    radio.begin();
    network.begin(/*channel*/ 90, /*node address*/ this_node);

    int i;
    intCmd = (char**)malloc(Ncommands*sizeof(char*));
    for(i=0;i<Ncommands;i++)
        intCmd[i] = (char*)malloc(MaxPayload*sizeof(char));
    setInternalCommands(intCmd); // robot commands
}

void loop(void)
{
    // Pump the network regularly
    network.update();
    communication(0);
}

void communication(bool mode) // mode 0: user written command; mode 1: std_command
{
	unsigned long t_aux = millis();
	bool w = 0,r = 0, quit = 0;

	while( (millis() < t_aux+t_com) && !(w&r) ) 
    {	
        if(reading)
        {
            if(Read_blocking())
            {
                r = 1;
                reading = 0;
                if(mode)
                    processing();
            }
        }
        else
        {
            if(Write_blocking(mode, other_node) )
            {
                w = 1;
                reading = 1;
            }
        }
    }
}

bool Read_blocking()
{
	bool r = Read_nonBlocking();
	unsigned int n = 0;
	t = millis();

	while(!r)
	{
		if(millis() > t+dt)
			deadlockHandling(n++);

		if(!reading)
			return 0;
		r = Read_nonBlocking();
	}
	return 1;
}

void deadlockHandling(unsigned int n)
{
    if(n < n_max)
    {
        t += (unsigned long)random(t_min, t_max);
        if(t%2)
        {
            reading = !reading;
        }
    }
}

bool Read_nonBlocking()
{
    reading = 1;
    if(!network.available())
        return 0;
    bool done = false;
    while (!done)
    {
        RF24NetworkHeader header;
        done = network.read(header,rcv,sizeof(rcv));
    }
    if(done)
    {
        Serial.print("Message received: ");
        Serial.println(rcv);
        return 1;
    }
}

bool Write_nonBlocking(bool mode, const uint16_t to_node) // mode 0: user written command; mode 1: std_command
{
	bool check;
	reading = 0;

        RF24NetworkHeader header(to_node);
	if(mode)
    {
        if(cmd[0] != '\0')
            check = network.write(header,cmd,strlen(cmd));
        else 
            return 1;
    }
	else
	{
        if(msg[0] != '\0')
        {
            check = network.write(header, msg, sizeof(msg));
        }
        else 
        {
            return 1;
        }
	}

	if(check)
		return 1;
	else
		return 0;
}

bool Write_blocking(bool mode, const uint16_t to_node) // mode 0: user written command; mode 1: std_command
{
    bool w;
    unsigned int n = 0;

    if(!mode)
        read_message();
    w = Write_nonBlocking(mode, to_node);
    t = millis();
    while(!w)
    {
        if(millis() > t+dt)
            deadlockHandling(n++);

        if(reading)
            return 0;
        w = Write_nonBlocking(mode, to_node);
    }
    if(mode)
        cmd[0] = '\0';
    else
        msg[0] = '\0';

    return 1;
}

void read_message()
{
    if(Serial.available() == 0 && msg[0] == '\0')
        msg[0] = '\0';
    else
    {
        delay(100);
        int aux = 0;
        while(Serial.available() > 0)
            msg[aux++] = Serial.read();
        msg[aux] = '\0';
    }
}

void copyString(char *o, char *c)
{
    int i;
    for(i = 0; i < MaxPayload; i++)
    {
       c[i] = o[i];
        if(o[i] == '\0')
            break;
    }
}

bool isSubstring(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
    {
        return 0;
    }	

	for(i = 0; i < MaxPayload; i++ )
	{
		if((a[i] == '\0')||(b[i] == '\0'))
			if((a[i] == '\0')&&(b[i] == '\0'))
				return 1;
			else
			{
				return 0;
			}
		if(a[i] != b[i])
			return 0;
	}
	return 1;
}

void setInternalCommands(char**m)
{

	copyString("write",m[0]);
	copyString("status",m[1]);
}

//TODO: return é realmente importante?
bool processing()
{
	int i;
	for(i = 0; i < Ncommands; i++) // have we just got a command?
		if(isSubstring(intCmd[i],rcv) == 1)
			break;

	if(i == Ncommands) // if not a command
    {
        if(wrt)
        {
            if(to_whom)
            {
                wrt = 0;
                to_whom = 0;
                write_command(rcv);
                Write_blocking(1,write_node);
                write_command("sent it!");
            }
            else
            {
                int number = char2int(rcv);
                if(number >= 0)
                {
                    write_node = number;
                    write_command("write message to node ");
                    concatenate(cmd, rcv);
                    to_whom = 1;
                }
            }
        }
        else // nothing recognizable send it back
            write_command(rcv);
    }
	else // perform command i
	{
		action(i);
	}

    return 0;
}

void action(int n)
{

    if(n < Ncommands)
        switch(n)
        {
            case 0:
                write_command("write to whom?");
                wrt = 1;
                break;

            case 1:
                write_command("ok");
                break;
        }
}

int mypow(int base, int exponent)
{
    int result = 1;
    for(int i = 0; i < exponent; i++)
        result = result*base;
    return result;
}

int char2int(char *s)
{
    int i = 0,j, r = 0, aux;

    for(i = 0; i < MaxPayload; i++)
        if(s[i] == '\0')
            break;

    if(i == MaxPayload)
    {
        write_command("-3"); // number too big for this application
        return -3;
    }

    if(i == 0)
    {
        write_command("-2"); // empty string
        return -2;
    }

    for(j=0; j < i; j++)
    {
        //if((s[j] >= '0') && (s[j] <= '9'))
        if((s[j] > 47) && (s[j] < 57))
        {
            aux = (int) s[j];
            aux -= 48;
            aux *= (int) mypow(10,i-1-j);
            r += aux;
        }
        else
        {
            int2char(s,i);
            concatenate(s,":error!");
            write_command(s);
            return -1;
        }
    }
    return r;
}

void int2char(char *o,int n)
{
	int i = 1, aux = 10, j;
	if(n >= 0)
	{
		while(n/i > 9)
			i *=10;
		for(j = 0; i>0; j++)
		{
			aux = n/i;
			o[j] = aux+48;
			n -= aux*i;
			i /= 10;
		}
		o[j] = '\0';

	}
	else // negative values aren't useful for this application
		o[0] = '\0';
}


void write_command(char *m)
{
	copyString(m,cmd);
}

void concatenate(char *o, char *p)
{
	int i,j,k;
	for(i = 0; i < MaxPayload; i++)
		if(o[i] == '\0')
			break;

	for(j = 0; j < MaxPayload; j++)
		if(p[j] == '\0')
			break;

	if( i+j <= MaxPayload)
	{
		for(k = 0; k < j ;k++)
			o[i+k] = p[k];
		o[i+k] = '\0';
	}
}

