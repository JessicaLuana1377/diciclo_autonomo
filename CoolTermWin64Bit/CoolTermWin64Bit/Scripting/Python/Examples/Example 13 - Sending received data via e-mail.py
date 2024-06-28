# Responding to received data
#
# This script grabs received data from the
# receive buffer at periodic intervals and
# sends it via e-mail.
#
# This examples uses Gmail, but this works
# for any SMTP e-mail service.
#
# Author: Roger Meier, 04-27-2023
# CoolTerm version: 2.0.1

# Credentials for SMTP server
SMTPserver = "smtp.gmail.com" # specific to Gmail
SMTPport = 587 # standard port for secure e-mail
username = "YOUR_GMAIL_ADDRESS"
password = "YOUR_GMAIL_APP_PASSWORD" # https://support.google.com/accounts/answer/185833?hl=en
receiverEmail = "RECIPIENT_EMAIL_ADDRESS"

import sys
import time
import smtplib
import CoolTerm
s = CoolTerm.CoolTermSocket()

period = 10 # check interval in seconds


# Get the ID of the frontmost open window
ID = s.GetFrontmostWindow()
if ID < 0:
    print("No open windows")
    sys.exit()

# Open the serial port
if not s.Connect(ID):
    print("Not connected")
    sys.exit()

# Monitor the receive buffer
try:

    while True: # endless loop
        # wait for the specified interval
        print("Waiting for incoming data...")
        time.sleep(10)
        
        # Check the receive buffer
        print("Checking receive buffer...")
        # Poll the port
        s.Poll(ID)
        # see if any data has arrived
        i = s.BytesAvailable(ID)
        print("Received {} bytes:".format(i))

        # Read the data from the receive buffer
        d = s.ReadAll(ID)
        print(d)

        # Send the data via e-mail
        if i > 0:
            print("Sending e-mail")
            try:
                # Set up e-mail
                m = smtplib.SMTP(SMTPserver, SMTPport)
                # start TLS for security
                m.starttls()
                # Log on to e-mail server
                m.login(username, password)
                # sending the mail
                m.sendmail(username, receiverEmail, d)
                print("Email sent successfully")
            except smtplib.SMTPConnectError:
                print("Error connecting to SMTP server")
            except smtplib.SMTPRecipientsRefused:
                print("Unable to send email to: " + receiverEmail)
            except smtplib.SMTPAuthenticationError:
                print("Authentication error when sending Email")
            # log off
            m.quit()
            print()
        

            
except KeyboardInterrupt:
    pass
    
# Close the port
s.Disconnect(ID)

# Disconnect from CoolTerm
s.Close()