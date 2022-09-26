# Directive header (time transmitted, frame of reference)
Header header
#
# Directive ID
int32 id
#
# Action name
string name
#
# Description of intent or purpose
string description
#
# Node source of the directive
string source
#
# Node destination of the directive
string destination
#
# Time to accept or reject directive
duration response_time
#
# Priority of the directive
int16 priority
#
# Params put in their own message class for now due to potential complexity
# May change this at a later date
Parameters params
