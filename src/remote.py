"""
Remote control with Firebase. Quite hacky and not particularly safe
"""
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import subprocess
import sys

CHANNEL_ID='1'
COLLECTION_ID='recording_control'
RECORDING_BIN='./target/rscapture'

# errors ignored, just does not play unless the sound and command both exist
PLAY_SOUND_CMD = ['paplay', '/usr/share/sounds/gnome/default/alerts/sonar.ogg']

# Use the application default credentialss
cred = credentials.Certificate('firebase-admin.json')
firebase_admin.initialize_app(cred)

db = firestore.client()
def play_sound():
    # open in the background to avoid blocking until the sound is played
    subprocess.Popen(PLAY_SOUND_CMD,
        stdout=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    # ignore any errors

class Recorder:
    def __init__(self):
        self.session = None

    def start(self):
        if self.session is not None:
            print("WARNING: already started")
            return

        play_sound()

        self.session = subprocess.Popen(RECORDING_BIN,
            stdout=sys.stdout,
            stdin=subprocess.PIPE,
            stderr=sys.stderr)

    def stop(self):
        if self.session is None:
            print("WARNING: already stopped")
            return

        play_sound()

        self.session.communicate(b'stop') # should kill the session
        self.session = None

recorder = Recorder()

def execute(command):
    if command == 'start': recorder.start()
    elif command == 'stop': recorder.stop()
    else: print('Invalid command received: {}'.format(command))

# Create a callback on_snapshot function to capture changes
def on_change(doc_snapshot, changes, read_time):
    for doc in doc_snapshot:
        print(u'Received document snapshot: {}, {}'.format(doc.id, doc.to_dict()))
        execute(doc.to_dict().get('command', ''))

doc_ref = db.collection(COLLECTION_ID).document(CHANNEL_ID)

# Watch the document
doc_ref.on_snapshot(on_change)

from time import sleep
while True: sleep(1)

#sleep(1)
#doc_ref.set({ 'command': 'start' })
#sleep(20)
#doc_ref.set({ 'command': 'stop' })
#sleep(1)
