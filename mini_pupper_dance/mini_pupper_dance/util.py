def parse_movement_string(movement_string):
    parts = movement_string.split(':')
    value = parts[1] if len(parts) > 1 and parts[1].lower() != 'off' else False
    interval = parts[2] if len(parts) > 2 else None

    action = parts[0]
    
    if action == 'music':
        value = True if value and value.lower() == 'on' else False

    elif action == 'volume':
        value = True if value and value.lower() == 'on' else False
        interval = int(parts[2]) 
    else:
        tm= value
        value = float(interval)
        interval = float(tm)
        
        
        
    if len(parts) == 3 or len(parts) == 2:
        action = parts[0]
        return {'action': action, 'value': interval, 'interval': value}
    else:
        raise ValueError("Invalid movement string format")

# Example usage:
movement_string = 'look_up:-0.3:0.5'
result = parse_movement_string(movement_string)
print(result)

command2 = 'music:on:robot1.wav'
result= parse_movement_string(command2)
print(result)
command2 = 'music:off'
result= parse_movement_string(command2)
print(result)
command3 = 'volume:on:100'
result= parse_movement_string(command3)
print(result)
