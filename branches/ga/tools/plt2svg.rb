#!/usr/bin/ruby

$input_file = ARGV[0]
$output_file = ARGV[1]

lines = File.open($input_file).read.chomp.split(";")

init_cmds = []


while cmd = lines.shift

	if cmd =~ /^PD/ || cmd =~ /^PU/
		lines.unshift(cmd)
		break
	end
	init_cmds << cmd
end


current_path = []
paths = []
current_x = 0
current_y = 0

while cmd = lines.shift
	if    cmd =~ /^PD([-\d]+),([-\d]+)/
		current_path << [$1 .to_i, $2 .to_i]
	elsif cmd =~ /^PU([-\d]+),([-\d]+)/
		if current_path.size > 1
			paths << current_path
		end
		current_path =  [[$1 .to_i, $2 .to_i]]
	elsif cmd =~ /^PU/
		if current_path.size > 1
			paths << current_path
		end
		current_path = []
	else
		if current_path.size > 1
			paths << current_path
		end
		break
	end
end

print <<END
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!-- Created with Inkscape (http://www.inkscape.org/) -->
<svg id="svg2" xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns="http://www.w3.org/2000/svg" height="297mm" width="210mm" version="1.1" xmlns:cc="http://creativecommons.org/ns#" xmlns:dc="http://purl.org/dc/elements/1.1/">
	<defs id="defs4">


		<marker id="Arrow1Mend" refY="0.0" refX="0.0" orient="auto">
			<path style="marker-start:none;" d="M0,0l5-5-17.5,5,17.5,5-5-5z" fill-rule="evenodd" transform="scale(0.4) rotate(180) translate(10,0)" stroke="#000" stroke-width="1.0pt"/>
		</marker>
		<marker id="DotS" refY="0.0" refX="0.0" orient="auto">
			<path style="marker-start:none;marker-end:none;" d="M-2.5-1c0,2.76-2.24,5-5,5s-5-2.24-5-5,2.24-5,5-5,5,2.24,5,5z" fill-rule="evenodd" transform="scale(0.2) translate(7.4, 1)" stroke="#000" stroke-width="1.0pt"/>
		</marker>

	</defs>
	<metadata id="metadata7">
		<rdf:RDF>
			<cc:Work rdf:about="">
				<dc:format>image/svg+xml</dc:format>
				<dc:type rdf:resource="http://purl.org/dc/dcmitype/StillImage"/>
				<dc:title/>
			</cc:Work>
		</rdf:RDF>
	</metadata>
	<g id="layer1">
END

index = 0
paths.each do |path|
	last_x = 0
	last_y = 0
	is_first = true
	path.each do | xy |
		x = xy[0].to_i
		y = xy[1].to_i
		if !is_first
			print <<-"END"
				<path
					id="path#{index}"
					stroke-linejoin="miter"
					style="marker-start:url(#DotS);marker-end:url(#Arrow1Mend);"
					d="M#{last_x},#{last_y},#{x},#{y}"
					stroke="#000"
					stroke-linecap="butt"
					stroke-width="1px"
					fill="none"/>


				<text
					id="text#{index}"
					style="writing-mode:lr-tb;letter-spacing:0px;
						text-anchor:middle;word-spacing:0px;text-align:center;"
					font-family="Sans"
					font-weight="normal"
					xml:space="preserve"
					stroke-dasharray="none"
					font-size="10px"
					font-style="normal"
					stroke="#00ffff"
					font-stretch="normal"
					stroke-miterlimit="4"
					font-variant="normal"
					x="#{(last_x+x)/2.0}"
					y="#{(last_y+y)/2.0}"
					stroke-width="0.3"
					line-height="125%"
					fill="#0000ff">
					<tspan 		
						x="#{(last_x+x)/2.0}"
						y="#{(last_y+y)/2.0}" >#{index}</tspan>
				</text>

			END
			index = index + 1
		end
		is_first = false
		last_x = x
		last_y = y
	end
end

print <<EOF
	</g>
</svg>
EOF
print "\n"

