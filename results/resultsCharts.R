

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


layout(matrix(c(1:36), 6, 6, byrow = TRUE))


allAgg <- c(0,0,0)

#~ pad <- 0.0625/4
#~ cWidth <- (1 - 4*pad)/3

qualCol <- "joint_score"

scores <- list()
cMar <- c(5,2,1,0.8)
#~ par(mar=cMar)
colors <- c("deepskyblue", "darkgreen", "burlywood3", "cyan4", "darkorange3", "darkolivegreen1", "purple")

for (chartNum in c(1:36)) {
	
	col <- chartNum %% 6
	if(col == 0){
		col <- 6
	}
	row <- ceiling(chartNum / 6)
	
	sceneNum <- floor((row-0.1)/2) * 3 + ceiling(col/2)
	scene <- paste0("scene", sceneNum)
	
	if(col %% 2 == 1) {
		# top row
		if(row %% 2 == 1) {
			# Availability
			passPct <- c()
			for (alg in algs) {
				runSet <- results[results$alg == alg & results$scene == scene,]
				passSet <- runSet[runSet$success == "true" & runSet$time < RT,]
				passPct <- c(passPct, nrow(passSet)/nrow(runSet))
			}
			scores[[sceneNum]] <- list()
			scores[[sceneNum]][["passPct"]] <- passPct
			names(passPct) <- algs
			barplot(passPct, col=colors, ylim=c(0,1))
			title(sub="Availability", line=2.5)
		}else{
			# Perf
			time <- c()
			timeMax <- c()
			timeMin <- c()
			
			for (alg in algs) {
				runSet <- results[results$alg == alg & results$scene == scene,]
				passSet <- runSet[runSet$success == "true" & runSet$time < RT,]
				if (nrow(passSet) > 0) {
					time <- c(time, summary(passSet$time)[["Median"]])
					timeMax <- c(timeMax, summary(passSet$time)[["Max."]])
					timeMin <- c(timeMin, summary(passSet$time)[["Min."]])
				}else{
					time <- c(time, -1)
					timeMax <- c(timeMax, -1)
					timeMin <- c(timeMin, -1)
				}
			}
			scores[[sceneNum]][["time"]] <- time
			names(time) <- algs
			newTime <- time
			newTime[time==-1] = 0
			time_plot <- barplot(newTime, col=colors, ylim=c(0,RT))
			title(sub="Performance", line=2.5)
			arrows(x0 = time_plot,                           # Add error bars
			   y0 = timeMax,
			   y1 = timeMin,
			   angle = 90,
			   code = 3,
			   length = 0.1)
		}
	}else{
		# bottom row
		if(row %% 2 == 1) {
			# Quality
			qual <- c()
			qualMax <- c()
			qualMin <- c()
			sceneBestQual <- min(results[results$success == "true" & results$scene == scene,qualCol])
			
			for (alg in algs) {
				runSet <- results[results$alg == alg & results$scene == scene,]
				passSet <- runSet[runSet$success == "true" & runSet$time < RT,]
				if (nrow(passSet) > 0) {
					invScore <- sceneBestQual/passSet[,qualCol]
					
					qual <- c(qual, summary(invScore)[["Median"]])
					qualMax <- c(qualMax, summary(invScore)[["Max."]])
					qualMin <- c(qualMin, summary(invScore)[["Min."]])
				}else{
					qual <- c(qual, -1)
					qualMax <- c(qualMax, -1)
					qualMin <- c(qualMin, -1)
				}
			}
			scores[[sceneNum]][["qual"]] <- qual
			names(qual) <- algs
			qual[qual==-1] = 0
			base_r_barplot <- barplot(qual, ylim = c(0, 1.0), col=colors)
			title(sub="Quality", line=2.5)
			arrows(x0 = base_r_barplot,                           # Add error bars
			   y0 = qualMax,
			   y1 = qualMin,
			   angle = 90,
			   code = 3,
			   length = 0.1)
		}else{
			# Aggregate
			time <- scores[[sceneNum]][["time"]]
			passPct <- scores[[sceneNum]][["passPct"]]
			qual <- scores[[sceneNum]][["qual"]]
			oldTime <- time
			names(passPct) <- algs
			names(qual) <- algs
			qual[qual==-1] = 0
			names(time) <- algs
			invTime <- (RT - time) / RT
			time[oldTime==-1] = 0
			total <- passPct*((qual + invTime)/2)
			barplot(total, ylim = c(0, 1), col=colors)
			title(sub="Aggregate", line=2.5)
		}
	}
}


dev.off()
