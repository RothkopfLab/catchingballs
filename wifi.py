import subprocess


def connect_to_network(name):
    process = subprocess.Popen(
        'netsh wlan connect {0}'.format(name),
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    # Return `True` if we were able to successfully connect
    return 'Die Verbindungsanforderung wurde erfolgreich abgeschlossen.' in stdout
