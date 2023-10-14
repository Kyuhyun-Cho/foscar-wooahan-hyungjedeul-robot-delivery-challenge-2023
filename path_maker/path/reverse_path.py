# 원본 파일 이름
input_filename = '/home/foscar/wooahan_challenge/catkin_ws/src/path_maker/path/outdoor_portal_5.txt'
# 새 파일 이름
output_filename = '/home/foscar/wooahan_challenge/catkin_ws/src/path_maker/path/outdoor_5_portal.txt'

# 원본 파일 열기
with open(input_filename, 'r') as input_file:
    # 파일 내용을 읽어옴
    lines = input_file.readlines()

# 내용을 뒤집음
lines.reverse()

# 새 파일에 내용을 쓰기
with open(output_filename, 'w') as output_file:
    output_file.writelines(lines)

print(f"'{input_filename}' 파일을 역순으로 뒤집어 '{output_filename}' 파일로 저장했습니다.")
