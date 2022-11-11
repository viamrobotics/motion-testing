

results <- read.csv("results.csv")
algs <- unique(results$alg)
algs <- algs[!grepl("ompl", algs, fixed = TRUE)]
algs <- algs[!grepl("cbrt_lin", algs, fixed = TRUE)]

#~ algs <- c("ompl2", "ompl5", "ompl10", "ompl20","ompl120" )
scenes <- unique(results$scene)
scenes <- scenes[!grepl("scene10", scenes, fixed = TRUE)]
scenes <- scenes[!grepl("scene11", scenes, fixed = TRUE)]
seeds <- unique(results$seed)

algs <- c("cbrt_fast_impr", "rrt*_1s", "rrt*_1.5s", "rrt*_5s", "ompl5")

rnum <- 3
cnum <- 1

first <- T
#~ png(filename =  "results.png",  width = 3000, height = 2000, pointsize = 16)

#~ algs <- c(algs, "ompl5")
RT<- 6
png(filename =  "5_seconds_results.png",  width = 3000, height = 2000, pointsize = 16)
#~ algs <- c(algs, "ompl20")
#~ RT<- 21
#~ png(filename =  "20_seconds_results.png",  width = 3000, height = 2000, pointsize = 16)


layout(matrix(c(1:9), 3, 3, byrow = TRUE))


allAgg <- c(0,0,0)

pad <- 0.0625/4
cWidth <- (1 - 4*pad)/3
for (scene in scenes) {
#~ 	print(scene)
#~ 	png(filename =  paste0("5_seconds_results", scene, ".png"),  width = 600, height = 600, pointsize = 14)
#~ 	layout(matrix(c(1:4), 2, 2, byrow = TRUE))
	time <- c()
	timeMax <- c()
	timeMin <- c()
	qual <- c()
	qualMax <- c()
	qualMin <- c()
	passPct <- c()
	total <- c()
	
	sceneBestQual <- min(results[results$success == "true" & results$scene == scene,"joint_score"])
	
	for (alg in algs) {
		runSet <- results[results$alg == alg & results$scene == scene,]
		
		passSet <- runSet[runSet$success == "true" & runSet$time < RT,]
		passPct <- c(passPct, nrow(passSet)/nrow(runSet))
		if (nrow(passSet) > 0) {
#~ 			invTime <- (RT - passSet$time) / RT
#~ 			time <- c(time, summary(invTime)[["Median"]])
#~ 			timeMax <- c(timeMax, summary(invTime)[["Max."]])
#~ 			timeMin <- c(timeMin, summary(invTime)[["Min."]])
			time <- c(time, summary(passSet$time)[["Median"]])
			timeMax <- c(timeMax, summary(passSet$time)[["Max."]])
			timeMin <- c(timeMin, summary(passSet$time)[["Min."]])
			
			invScore <- sceneBestQual/passSet$joint_score
			
			qual <- c(qual, summary(invScore)[["Median"]])
			qualMax <- c(qualMax, summary(invScore)[["Max."]])
			qualMin <- c(qualMin, summary(invScore)[["Min."]])
			
		}else{
			time <- c(time, -1)
			timeMax <- c(timeMax, -1)
			timeMin <- c(timeMin, -1)
#~ 			timeSD <- c(timeSD, -1)
			qual <- c(qual, -1)
			qualMax <- c(qualMax, -1)
			qualMin <- c(qualMin, -1)
#~ 			qualSD <- c(qualSD, -1)
		}
	}
	
	# in case there is only one pass
	
	oldTime <- time
	names(passPct) <- algs
#~ 	names(qualSD) <- algs
	names(qual) <- algs
#~ 	qual <- (1.5*max(qual + qualSD))-qual
	qual[qual==-1] = 0
#~ 	names(timeSD) <- algs
	names(time) <- algs
#~ 	time <- (1.5*max(time))-time
	invTime <- (RT - time) / RT
	time[oldTime==-1] = 0
	

	
#~ 	total <- (passPct*2 + qual*2 + time)/5
	total <- passPct*((qual + invTime)/2)
#~ 	total <- passPct*qual
	
	allAgg <- allAgg + total

	cMar <- c(5,2,1,0.8)
	cpad <- pad*cnum + cWidth*(cnum-1)
	rpad <- pad*rnum + cWidth*(rnum-1)
	colors <- c("deepskyblue", "darkgreen", "burlywood3", "cyan4", "darkorange3", "darkolivegreen1", "purple")
	
	if (first== T) {
		par(fig=c(cpad,cWidth/2+cpad,rpad,cWidth/2+rpad), mar=cMar)
		first <- F
	}else{
		par(fig=c(cpad,cWidth/2+cpad,rpad,cWidth/2+rpad), mar=cMar, new=T)
	}
#~ 	par(mar=cMar)
	base_r_barplot <- barplot(qual, ylim = c(0, 1.0), col=colors)
	title(sub="Quality", line=2.5)
	arrows(x0 = base_r_barplot,                           # Add error bars
	   y0 = qualMax,
	   y1 = qualMin,
	   angle = 90,
	   code = 3,
	   length = 0.1)
#~ 	par(mar=cMar)
	

#~ 	par(mar=cMar)
	par(fig=c(0.005+cWidth/2+cpad,cWidth+cpad,cWidth/2+rpad,cWidth+rpad), new=TRUE, mar=cMar)
	time_plot <- barplot(time, col=colors, ylim=c(0,RT))
	title(sub="Performance", line=2.5)
	arrows(x0 = time_plot,                           # Add error bars
	   y0 = timeMax,
	   y1 = timeMin,
	   angle = 90,
	   code = 3,
	   length = 0.1)
	
#~ 	par(mar=cMar)

	print(qual)
	
	par(fig=c(cpad,cWidth/2+cpad,cWidth/2+rpad,cWidth+rpad), new=TRUE, mar=cMar)
	barplot(passPct, col=colors, ylim=c(0,1))
	title(sub="Availability", line=2.5)

	par(fig=c(cWidth/2+cpad,cWidth+cpad,rpad,cWidth/2+rpad), mar=cMar, new=T)
	barplot(total, ylim = c(0, 1), col=colors)
	title(sub="Aggregate", line=2.5)
	
	cnum <- cnum + 1
	if (cnum > 3) {
		rnum <- rnum - 1
		cnum <- 1
	}
#~ 	dev.off()
}
#~ par(xpd=NA)
#~ abline(v=-17.25, lwd=3)
#~ abline(v=-48.75, lwd=3)
#~ abline(h=22, lwd=3)
#~ abline(h=77, lwd=3)

dev.off()

png(filename =  "overall5_seconds_results.png",  width = 1000, height = 1000, pointsize = 16)
barplot(allAgg/9, main="Overall", sub="Aggregate", ylim = c(0, 1.0), col=colors)
dev.off()
