CC = 'clang'
CFLAGS = []
LDFLAGS = []
APPLICATION = 'program'
OBJDIR = 'obj'
INC_DIRS = FileList['**/inc/']
C_FILES = FileList['**/src/*.c']
H_FILES = FileList['**/inc/*.h']

CFLAGS << [ INC_DIRS.collect {|d| "-I"+d } ]

OBJ_FILES = C_FILES.collect { |fn|
    ofile = File.join(OBJDIR, File.basename(fn).ext('o'))
    file ofile => [OBJDIR, fn] do
        sh "#{CC} #{CFLAGS.join ' '} -o #{ofile} -c #{fn}"
    end
    ofile
}

directory OBJDIR

file APPLICATION => OBJ_FILES do |t|
    sh "#{CC} #{CFLAGS.join ' '} #{LDFLAGS.join ' '} -o #{t.name} #{OBJ_FILES.join ' '}"
end

task :default => [APPLICATION]
