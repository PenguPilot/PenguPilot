#!/usr/bin/env python
#
# iwlistparse.py
# Hugo Chargois - 17 jan. 2010 - v.0.1
# Parses the output of iwlist scan into a table

# Above is the original copyright notice from the author
# Below is mine!

# iwlistparse2.py
# Modified by Vuong Do Thanh Huy - 21 April 2012 - v.1.0
# This is the version for Netbook with small screen,
#  since it only outputs the max bit rates, not the full list of bit rates.

# Source of original version: https://bbs.archlinux.org/viewtopic.php?id=88967




import re

# You can add or change the functions to parse the properties of each AP (cell)
# below. They take one argument, the bunch of text describing one cell in iwlist
# scan and return a property of that cell.

def get_name(cell):
    return matching_line(cell,"ESSID:")[1:-1]

def get_quality(cell):
    quality = matching_line(cell,"Quality=").split()[0].split('/')
    return str(int(round(float(quality[0]) / float(quality[1]) * 100))).rjust(3) + " %"

def get_channel(cell):
    return matching_line(cell,"Channel:")

# This function is modified by Vuong Do Thanh Huy to display more information.
def get_encryption(cell):
    # Get encryption
    enc=""
    if matching_line(cell,"Encryption key:") == "off":
        enc="Open"
    else:
        for line in cell:
            matching = match(line,"IE:")
            if matching!=None:
                wpa=match(matching,"WPA Version ")
                if wpa!=None:
                    #enc="WPA v."+wpa
                    enc="WPA"
                wpa2=match(matching,"WPA2 Version ")
                if wpa2!=None:
                    #enc="WPA2 v."+wpa2
                    enc="WPA2"
        if enc=="":
            enc="WEP"
    # Get authetication type
    authen = ""
    authen = matching_line(cell,"Authentication Suites (1) : ")

    # Get cipher type
    cipher1 = ""
    cipher2 = ""
    cipher1 = matching_line(cell,"Group Cipher : ")
    cipher2 = matching_line(cell,"Pairwise Ciphers (1) : ")

    result = enc + " - " + authen + " || " + cipher1 + " - " + cipher2
    return result

def get_address(cell):
    return matching_line(cell,"Address: ")

# These functions are added by Vuong Do Thanh Huy to display more information.
# Modify it as you need.

# Because this function returns a long list of bit rates,
#  I have made it only print the maximum one,
#  for the purpose of suiting the small netbook screen.
def getCellBitRates(cell):
    search = matching_line(cell,"Bit Rates:")
    for line in cell:
        string = str(line)
        matchObj = re.match(r"[ ]{20}\d+",string,flags=0)
        if matchObj:
            search = search + str(line)
    preline = ""
    for line in cell:
        string = str(line)
        matchObj = re.search(r"Mode:",string,flags=0)
        if matchObj:
            break
        else:
            preline = string
    search = search + preline
    bitrate = search.split('; ')
    result = []
    for i in range(0,len(bitrate)):
        result.append(float(bitrate[i].split(' Mb/s')[0]))
    result.sort()
    # This line below only gets the maximum value of bit rates.
    output = str(result[len(result)-1])
    # This block gets all values of bit rates.
    # To use this, comment out the above line, and uncomment those line below.
#    for i in range(0,len(result)):
#        if i!=len(result)-1:
#            output = output + str(result[i]) + "|"
#        else:
#            output = output + str(result[i])
    output = output + " (Mb/s)"
    return output

def getCellFrequency(cell):
    freq = matching_line(cell,"Frequency:").split()[0].split('GHz')
    return freq[0] + "GHz"

def getCellSignal(cell):
    return matching_line(cell,"Quality=").split('Signal level=')[1]

def getCellMode(cell):
    return matching_line(cell,"Mode:")

def getLastBeacon(cell):
    return matching_line(cell,"Extra: Last beacon: ")




# Here's a dictionary of rules that will be applied to the description of each
# cell. The key will be the name of the column in the table. The value is a
# function defined above.

# Modified by Vuong Do Thanh Huy, to add more keys.
rules={"Name":get_name,
       "Quality":get_quality,
       "Ch":get_channel,
       #"Encryption":get_encryption,
       "Address":get_address,
       "Frequency":getCellFrequency,
       "Signal":getCellSignal,
       "Mode":getCellMode,
       "Bit Rates":getCellBitRates,
       "Last Beacon":getLastBeacon,
       }

# Here you can choose the way of sorting the table. sortby should be a key of
# the dictionary rules.

def sort_cells(cells):
    sortby = "Quality"
    reverse = True
    cells.sort(None, lambda el:el[sortby], reverse)

# You can choose which columns to display here, and most importantly in what order. Of
# course, they must exist as keys in the dict rules.

#Modified by Vuong Do Thanh Huy, to display more information.
columns=["Name","Address","Quality","Signal","Frequency","Ch","Mode","Last Beacon","Bit Rates"]




# Below here goes the boring stuff. You shouldn't have to edit anything below
# this point

def matching_line(lines, keyword):
    """Returns the first matching line in a list of lines. See match()"""
    for line in lines:
        matching=match(line,keyword)
        if matching!=None:
            return matching
    return None

def match(line,keyword):
    """If the first part of line (modulo blanks) matches keyword,
    returns the end of that line. Otherwise returns None"""
    line=line.lstrip()
    length=len(keyword)
    if line[:length] == keyword:
        return line[length:]
    else:
        return None

def parse_cell(cell):
    """Applies the rules to the bunch of text describing a cell and returns the
    corresponding dictionary"""
    parsed_cell={}
    for key in rules:
        rule=rules[key]
        parsed_cell.update({key:rule(cell)})
    return parsed_cell

def print_table(table):
    widths=map(max,map(lambda l:map(len,l),zip(*table))) #functional magic

    justified_table = []
    for line in table:
        justified_line=[]
        for i,el in enumerate(line):
            justified_line.append(el.ljust(widths[i]+2))
        justified_table.append(justified_line)
    
    for line in justified_table:
        for el in line:
            print el,
        print

def print_cells(cells):
    table=[columns]
    for cell in cells:
        cell_properties=[]
        for column in columns:
            cell_properties.append(cell[column])
        table.append(cell_properties)
    print_table(table)

def parse_cells(lines):
    cells=[[]]
    parsed_cells=[]

    for line in lines:
        cell_line = match(line,"Cell ")
        if cell_line != None:
            cells.append([])
            line = cell_line[-27:]
        cells[-1].append(line.rstrip())

    cells=cells[1:]

    for cell in cells:
        parsed_cells.append(parse_cell(cell))
    return parsed_cells

if __name__ == '__main__':
    import sys
    parsed_cells = parse_cells(sys.stdin.readlines())
    sort_cells(parsed_cells)
    print_cells(parsed_cells)
