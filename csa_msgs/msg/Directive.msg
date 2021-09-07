# Directive header (directive ID, time transmitted, frame of reference)
Header header
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
# Time to accept or reject directive
duration response_time
#
# Priority of the directive
int16 priority
#
# Deadline for completion
time deadline
#
# The following are represented as strings, which are parsed at the end point component
# May change this at a later date
# Entry conditions
string[] entry_conds
#
# Exit (end, goals) conditions
string[] end_conds
# 
# Rules for the action
string[] rules
#
# Performance criteria placed on the action
string[] crits
