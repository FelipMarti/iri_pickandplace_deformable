#include "pickandplace_deformable_alg.h"

PickandplaceDeformableAlgorithm::PickandplaceDeformableAlgorithm(void)
{
	pthread_mutex_init(&this->access_, NULL);
}

PickandplaceDeformableAlgorithm::~PickandplaceDeformableAlgorithm(void)
{
	pthread_mutex_destroy(&this->access_);
}

void PickandplaceDeformableAlgorithm::config_update(Config & new_cfg,
						    uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// PickandplaceDeformableAlgorithm Public API
