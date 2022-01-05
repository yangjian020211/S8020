int errno = 0;

int * __errno()
{
    return &errno;
}
