awk '{printf $2$1"\n"$4$3"\n"$6$5"\n"$8$7"\n"$10$9"\n"$12$11"\n"$14$13"\n"$16$15"\n"}' $1 > $1_out
